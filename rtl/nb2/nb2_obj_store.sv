// Namco NB-2 MiSTer core -- DDR3 sprite-graphics store (region OBJ, docs/ARCHITECTURE.md section 2.2).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The 20 MiB C355 graphics (MAME region c355spr, first 20 MiB) live in the HPS DDR3 at byte BASE, reached through
// the core's own f2sdram port (DDRAM_*, Avalon-MM, 64-bit words, clocked by clk_sys). Two client ports with the
// NB-1 client contract:
//   L  loader / ROM checker: reads and writes (32-bit, big-endian lanes, byte enables), one outstanding;
//      an offset beyond the window answers rsp_err.
//   S  sprite renderer: 8-byte line reads, up to S_MAX outstanding, answered in request order. Offsets beyond the
//      20 MiB window read 0 without a DDR3 access (MAME's region is 32 MiB, zero-filled past the ROMs).
// Responses: line = the aligned 8-byte line {byte +0 .. +7}; rdata = the aligned longword at the offset.
// DDR3 byte order: DDRAM_DOUT[8k+7:8k] = byte 8*address + k, so the big-endian line is the byte-reversed word.
module nb2_obj_store #(
    parameter [31:0] BASE   = 32'h3200_0000,     // DDR3 byte address of offset 0
    parameter [25:0] WINDOW = 26'h1400000,
    parameter int    S_MAX  = 4
) (
    input  wire        clk_sys,
    input  wire        reset,                   // forget queued work (not the DDR3 contents)

    // port L
    input  wire        l_valid,
    output wire        l_ready,
    input  wire [25:0] l_offset,
    input  wire        l_we,
    input  wire [31:0] l_wdata,
    input  wire [3:0]  l_be,
    output reg         l_rsp_valid = 1'b0,
    output reg  [31:0] l_rsp_rdata = '0,
    output reg  [63:0] l_rsp_line = '0,
    output reg         l_rsp_err = 1'b0,

    // port S
    input  wire        s_valid,
    output wire        s_ready,
    input  wire [25:0] s_offset,
    output reg         s_rsp_valid = 1'b0,
    output reg  [63:0] s_rsp_line = '0,

    // DDR3 (DDRAM_* of the emu module)
    input  wire        DDRAM_BUSY,
    output wire [7:0]  DDRAM_BURSTCNT,
    output reg  [28:0] DDRAM_ADDR = '0,
    input  wire [63:0] DDRAM_DOUT,
    input  wire        DDRAM_DOUT_READY,
    output reg         DDRAM_RD = 1'b0,
    output reg  [63:0] DDRAM_DIN = '0,
    output reg  [7:0]  DDRAM_BE = '0,
    output reg         DDRAM_WE = 1'b0,

    output reg  [15:0] max_latency = '0          // longest S read (clk_sys), for diagnostics
);
    assign DDRAM_BURSTCNT = 8'd1;

    // in-order answer queue: {port S?, kind: 0 DDR3 read, 1 zero / write ack / error, err, off2}
    localparam int QD = 8;
    reg [3:0] q [0:QD-1];
    reg [2:0] q_rd = '0, q_wr = '0;
    reg [3:0] q_cnt = '0;
    reg [2:0] s_out = '0;                         // S requests not yet answered
    reg       l_out = 1'b0;
    // DDR3 read data returned but not yet answered (the head may be a non-DDR entry)
    reg [63:0] d [0:QD-1];
    reg [2:0]  d_rd = '0, d_wr = '0;
    reg [3:0]  d_cnt = '0;
    reg [3:0]  ddr_pend = '0;                     // DDR3 reads issued, data not yet returned
    reg [3:0]  drop = '0;                         // data of reads issued before a reset, to be discarded

    wire cmd_busy = (DDRAM_RD || DDRAM_WE) && DDRAM_BUSY;
    wire room     = (q_cnt < QD - 1) && !cmd_busy && !reset;
    wire l_go     = l_valid && !l_out && room;
    wire s_go     = !l_go && s_valid && (s_out < S_MAX) && room;
    assign l_ready = l_go;
    assign s_ready = s_go;

    wire        go_we  = l_go && l_we;
    wire [25:0] go_off = l_go ? l_offset : s_offset;
    wire        go_oob = (go_off >= WINDOW);
    wire        go_ddr = !go_oob;                 // a DDR3 command is issued
    wire [28:0] go_word = BASE[31:3] + {6'd0, go_off[25:3]};

    function automatic [63:0] rev8(input [63:0] v);
        for (int k = 0; k < 8; k++) rev8[63 - 8*k -: 8] = v[8*k +: 8];
    endfunction

    wire [3:0] head = q[q_rd];
    wire [63:0] head_line = rev8(d[d_rd]);
    wire head_ddr = (q_cnt != 0) && !head[2];
    wire head_now = (q_cnt != 0) && (head[2] || (d_cnt != 0));

    always @(posedge clk_sys) begin
        l_rsp_valid <= 1'b0;
        s_rsp_valid <= 1'b0;
        if (!DDRAM_BUSY) begin DDRAM_RD <= 1'b0; DDRAM_WE <= 1'b0; end

        // ---- issue
        if (l_go || s_go) begin
            q[q_wr] <= {s_go, !(go_ddr && !go_we), l_go && go_oob, go_off[2]};
            q_wr <= q_wr + 3'd1;
            if (go_ddr) begin
                DDRAM_ADDR <= go_word;
                if (go_we) begin
                    DDRAM_WE  <= 1'b1;
                    DDRAM_DIN <= {2{l_wdata[7:0], l_wdata[15:8], l_wdata[23:16], l_wdata[31:24]}};
                    DDRAM_BE  <= go_off[2] ? {l_be[0], l_be[1], l_be[2], l_be[3], 4'b0000}
                                           : {4'b0000, l_be[0], l_be[1], l_be[2], l_be[3]};
                end else begin
                    DDRAM_RD <= 1'b1;
                end
            end
        end
        // ---- DDR3 read data
                if (DDRAM_DOUT_READY && drop == 0) begin
            d[d_wr] <= DDRAM_DOUT;
            d_wr <= d_wr + 3'd1;
        end
        if (DDRAM_DOUT_READY && drop != 0) drop <= drop - 4'd1;
        ddr_pend <= ddr_pend + (((l_go || s_go) && go_ddr && !go_we) ? 4'd1 : 4'd0) - (DDRAM_DOUT_READY ? 4'd1 : 4'd0);
        // ---- answer the head
        if (head_now) begin
            if (head[3]) begin
                s_rsp_valid <= 1'b1;
                s_rsp_line  <= head[2] ? 64'd0 : head_line;
            end else begin
                l_rsp_valid <= 1'b1;
                l_rsp_err   <= head[1];
                l_rsp_line  <= head[2] ? 64'd0 : head_line;
                l_rsp_rdata <= head[2] ? 32'd0 : (head[0] ? head_line[31:0] : head_line[63:32]);
            end
            q_rd <= q_rd + 3'd1;
            if (!head[2]) d_rd <= d_rd + 3'd1;
        end
        q_cnt   <= q_cnt + ((l_go || s_go) ? 4'd1 : 4'd0) - (head_now ? 4'd1 : 4'd0);
        d_cnt   <= d_cnt + ((DDRAM_DOUT_READY && drop == 0) ? 4'd1 : 4'd0) - ((head_now && !head[2]) ? 4'd1 : 4'd0);
        s_out   <= s_out + (s_go ? 3'd1 : 3'd0) - ((head_now && head[3]) ? 3'd1 : 3'd0);
        if (l_go) l_out <= 1'b1;
        else if (head_now && !head[3]) l_out <= 1'b0;

        if (reset) begin
            // forget every queued answer; DDR3 reads still in flight are counted and their data discarded
            q_rd <= '0; q_wr <= '0; q_cnt <= '0; s_out <= '0; l_out <= 1'b0; d_rd <= '0; d_wr <= '0; d_cnt <= '0;
            // every read still in flight (ddr_pend counts them all, dropped or not) is discarded on arrival
            drop <= ddr_pend - (DDRAM_DOUT_READY ? 4'd1 : 4'd0);
        end
    end

    // diagnostics: S read latency (issue to answer of the oldest S request)
    reg [15:0] lat = '0;
    always @(posedge clk_sys) begin
        if (s_out == 0) lat <= '0;
        else lat <= (lat == 16'hFFFF) ? lat : lat + 16'd1;
        if (head_now && head[3]) begin
            if (lat > max_latency) max_latency <= lat;
            lat <= '0;
        end
        if (reset) max_latency <= '0;
    end
endmodule
