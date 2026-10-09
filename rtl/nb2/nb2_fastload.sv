// Namco NB-2 MiSTer core -- DDR3 fast ROM loading.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// With `address="0x30000000"` on the MRA's index-0 <rom>, Main_MiSTer (support/arcade/mra_loader.cpp rom_finish)
// does not stream the ROM through ioctl: it starts the download with the length in ioctl_addr
// (user_io_set_download(1, len)), copies the whole assembled stream into DDR3 at that address (shmem_put), and
// ends the download -- no ioctl_wr in between. Older firmware ignores the attribute and streams as before; both
// paths load the same bytes.
//
// This block sits between hps_io and the core. Normal downloads pass straight through. A fast download (index
// INDEX starting with ioctl_addr != 0) passes through as an empty download; when it ends, the block replays the
// stream from DDR3 into the core's ioctl inputs exactly as hps_io would deliver it (WIDE = 1: dout[7:0] = byte 2k,
// dout[15:8] = byte 2k+1, one word per ioctl_wr, the next only after ioctl_wait drops), keeping ioctl_download
// high without a gap so the core stays in reset. So the ROM loader, the C75 BIOS capture and the ROM check work
// unchanged.
// Only the SDRAM part [0, SD_END) is replayed. The sprite part of the stream (offset 0x2000000 on) is already
// where the sprite store reads it (0x30000000 + 0x2000000 = 0x32000000); of it only the stream's last word is
// replayed, so the loader records the full stream length the ROM check expects (it rewrites that word with the
// value just read). The filler between SD_END and 0x2000000 is skipped.
//
// DDR3: while it fetches, the block owns the core's DDR3 command port (own = 1; the sprite store, idle during a
// load, sees BUSY); it releases the port when every fetch has returned, before the last word is replayed (that
// write goes through the sprite store).
module nb2_fastload #(
    parameter [15:0] INDEX    = 16'd0,
    parameter [28:0] BASE_W   = 29'h6000000,    // DDR3 64-bit word of stream offset 0 (byte 0x30000000)
    parameter [26:0] SD_END   = 27'h1E84000,    // end of the SDRAM part of the stream (C75 BIOS window end)
    parameter int    DEPTH    = 4               // fetched lines in flight + buffered
) (
    input  wire        clk,

    // hps_io side
    input  wire        h_download,
    input  wire [15:0] h_index,
    input  wire        h_wr,
    input  wire [26:0] h_addr,
    input  wire [15:0] h_dout,
    output wire        h_wait,

    // core side (what the core's ioctl consumers see)
    output wire        c_download,
    output wire [15:0] c_index,
    output wire        c_wr,
    output wire [26:0] c_addr,
    output wire [15:0] c_dout,
    input  wire        c_wait,

    // DDR3 command port while own = 1 (BURSTCNT 1, read only)
    output reg         own = 1'b0,
    output reg         d_rd = 1'b0,
    output reg  [28:0] d_addr = '0,
    input  wire        d_busy,
    input  wire [63:0] d_dout,
    input  wire        d_dout_ready,

    output wire        active,                  // a replay is running
    output reg  [15:0] loads = '0               // fast loads completed (diagnostics)
);
    // ------------------------------------------------------------------ detect
    reg        dl_q = 1'b0;
    reg        fast = 1'b0;                     // the current index-INDEX download is a fast one
    reg [26:0] len = '0;
    reg        replay = 1'b0;
    always @(posedge clk) dl_q <= h_download;
    wire dl_rise = h_download && !dl_q;
    wire dl_fall = !h_download && dl_q;

    // replay ranges: words [0, sd_end) then (if the stream is longer) the single word at last_w
    wire [26:0] sd_end = (len < SD_END) ? len : SD_END;
    wire [26:0] last_w = len - 27'd2;
    wire        tail   = (len > SD_END);

    // ------------------------------------------------------------------ fetch (DDR3 -> line FIFO)
    localparam int CW = $clog2(DEPTH + 1);
    (* ramstyle = "logic" *) reg [63:0] fifo [0:DEPTH-1];   // registers: block RAM is nearly full
    reg [$clog2(DEPTH)-1:0] f_wr = '0, f_rd = '0;
    reg [CW-1:0] f_cnt = '0;                    // lines buffered
    reg [CW-1:0] f_out = '0;                    // reads issued, data not yet returned
    reg [26:0]   fetch_a = '0;                  // next line to fetch (byte offset, 8-aligned)
    reg          fetch_tail = 1'b0;             // the tail line is still to fetch
    wire fetch_more = (fetch_a < sd_end) || fetch_tail;
    wire [26:0] fetch_line = (fetch_a < sd_end) ? fetch_a : {last_w[26:3], 3'd0};
    wire room  = (f_cnt + f_out) < DEPTH;
    wire issue = replay && own && !d_rd && fetch_more && room;
    wire pop;

    always @(posedge clk) begin
        if (d_rd && !d_busy) d_rd <= 1'b0;
        if (issue) begin
            d_rd   <= 1'b1;
            d_addr <= BASE_W + {5'd0, fetch_line[26:3]};
            if (fetch_a < sd_end) fetch_a <= fetch_a + 27'd8;
            else fetch_tail <= 1'b0;
        end
        if (d_dout_ready && own) begin
            fifo[f_wr] <= d_dout;
            f_wr <= f_wr + 1'd1;
        end
        f_out <= f_out + (issue ? 1'd1 : 1'd0) - ((d_dout_ready && own) ? 1'd1 : 1'd0);
        f_cnt <= f_cnt + ((d_dout_ready && own) ? 1'd1 : 1'd0) - (pop ? 1'd1 : 1'd0);
        if (pop) f_rd <= f_rd + 1'd1;
        // release the port once every fetch is issued and back
        if (own && replay && !fetch_more && f_out == 0 && !d_rd && !issue) own <= 1'b0;
        if (dl_fall && fast) begin
            fetch_a <= '0; fetch_tail <= tail; f_wr <= '0; f_rd <= '0; f_cnt <= '0; f_out <= '0;
            own <= 1'b1;
        end
    end

    // ------------------------------------------------------------------ replay (line FIFO -> core ioctl)
    reg  [26:0] rp_a = '0;                      // next word to replay
    reg  [1:0]  rp_j = '0;                      // word within the head line
    reg         rp_tail = 1'b0;                 // only the tail word is left
    reg         wr = 1'b0;
    reg  [26:0] wr_a = '0;
    reg  [15:0] wr_d = '0;
    wire [63:0] head = fifo[f_rd];
    wire [1:0]  jj   = rp_tail ? last_w[2:1] : rp_j;
    wire        can  = replay && (f_cnt != 0) && !wr && !c_wait;
    wire        last_of_line = rp_tail || (rp_j == 2'd3) || (rp_a + 27'd2 >= sd_end);
    assign pop = can && last_of_line;

    always @(posedge clk) begin
        wr <= 1'b0;
        if (can) begin
            wr   <= 1'b1;
            wr_a <= rp_tail ? last_w : rp_a;
            wr_d <= head[16*jj +: 16];
            if (rp_tail) begin
                rp_tail <= 1'b0;
                replay  <= 1'b0;                // done (ioctl_download falls once the write is answered)
                loads   <= loads + 16'd1;
            end else begin
                rp_a <= rp_a + 27'd2;
                rp_j <= rp_j + 2'd1;
                if (rp_a + 27'd2 >= sd_end) begin
                    rp_j <= 2'd0;
                    if (tail) rp_tail <= 1'b1;
                    else begin replay <= 1'b0; loads <= loads + 16'd1; end
                end
            end
        end
        if (dl_rise && h_index == INDEX) begin
            fast <= (h_addr != 27'd0);
            len  <= h_addr;
        end
        if (dl_fall && fast) begin
            replay <= 1'b1; rp_a <= '0; rp_j <= '0; rp_tail <= 1'b0;
            fast <= 1'b0;
        end
    end

    // the core stays in download until the last replayed write has been answered
    reg tail_wait = 1'b0;
    always @(posedge clk) tail_wait <= replay || wr || (tail_wait && c_wait);
    assign active = replay || wr || tail_wait;

    assign c_download = h_download || dl_q || active;
    assign c_index    = active ? INDEX : h_index;
    assign c_wr       = active ? wr    : h_wr;
    assign c_addr     = active ? wr_a  : h_addr;
    assign c_dout     = active ? wr_d  : h_dout;
    assign h_wait     = c_wait;
endmodule
