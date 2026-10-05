// Namco NB-1 MiSTer core -- native 180-degree picture through a DDR3 frame buffer (M17).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// OSD Orientation = Flipped on the NATIVE raster (15-kHz analog, Direct Video and, through the normal video
// path, HDMI). On the NB-1 the picture cannot be flipped in the renderers: display line 0 would need game line
// 223, whose C123/C116 state (M13 per-line raster effects) exists only at the end of the frame. So each
// finished frame (game picture + overlay, as sent to the display) is written to DDR3 and shown one frame
// later, read bottom line first and each line right to left, on the unchanged native raster (the syncs,
// blanks, pixel clock and everything upstream are untouched). Owner decision, docs/M17_IMPLEMENTATION.md.
//
//   frame F:   pixels (x, y) captured at ce_pix while de, 2 per 64-bit word, into buffer F[0]
//   frame F+1: display (x, y) <- stored (287 - x, 223 - y) of frame F; frame F+1 written to the other buffer
//
// Latency: one frame (16.8 ms) in Flipped only. The first frame after Flipped is selected is black.
// Storage: DDR3 byte 0x30000000 (buffer 0) and 0x30080000 (buffer 1), 144 words x 224 lines = 252 KiB each
// (the framework uses 0x20000000.. for the HDMI scaler and 0x24000000.. for screen_rotate).
// No block RAM: the 64-word write FIFO and the 2 x 144-word line buffers are MLAB (ramstyle below).
//
// DDR3 port (the emu DDRAM_* Avalon master, clk_sys = DDRAM_CLK): commands held until !ddr_busy; one
// single-beat write per stored word (one per 32 clk), one read of 144 words per line as two 72-beat
// bursts, issued at the start of the line BEFORE the one that shows it (6,144 clk of slack). Reads go
// first; writes queue in the FIFO meanwhile. The port is shared with screen_rotate (HDMI CW/CCW), which
// writes only while its FB_EN is high; this module issues nothing unless `own` (NB1.sv mux).
module nb1_native_flip #(
    parameter [28:0] BASE0 = 29'h0600_0000,   // 64-bit word address of byte 0x30000000
    parameter [28:0] BASE1 = 29'h0601_0000    // byte 0x30080000
) (
    input  wire        clk_sys,
    input  wire        ce_pix,
    input  wire [8:0]  hcount,
    input  wire [8:0]  vcount,
    input  wire        de,
    input  wire        line_end,
    input  wire        frame_swap,     // start of vertical blank (capture of lines 0..223 done)
    input  wire [23:0] rgb_in,
    input  wire        flip_req,       // OSD Orientation = Flipped (any time; taken at the frame boundary)
    input  wire        own,            // the DDR3 port is ours (screen_rotate idle)

    output reg  [23:0] rgb_out = '0,   // flipped picture, registered (NB1.sv muxes it in while flipping)
    output reg         flipping = 1'b0,
    output wire        ddr_active,     // flipping, or a DDR3 transaction still in progress
    output reg  [15:0] late_lines = '0,  // a line not fetched in time (shown black)
    output reg  [15:0] fifo_drops = '0,  // write FIFO full (pixels lost)

    output wire [7:0]  ddr_burstcnt,
    output wire [28:0] ddr_addr,
    output wire [63:0] ddr_din,
    output wire [7:0]  ddr_be,
    output wire        ddr_we,
    output wire        ddr_rd,
    input  wire        ddr_busy,
    input  wire [63:0] ddr_dout,
    input  wire        ddr_dout_ready
);
    // ---------------------------------------------------------------- mode, taken at the frame boundary
    // At the start of vertical blank the frame just captured (lines 0..223) becomes the one shown next
    // (its last FIFO words drain long before line 223 of it is fetched, during line 263), and flipping
    // follows the OSD: changes land in vertical blank, never mid-picture.
    reg        wbuf = 1'b0;            // buffer being written
    reg        rd_valid = 1'b0;        // the buffer being read holds a frame captured whole
    always @(posedge clk_sys) if (frame_swap) begin
        wbuf     <= ~wbuf;
        rd_valid <= flipping;          // the frame just captured was captured while flipping
        flipping <= flip_req && own;
    end
    wire [28:0] wbase = wbuf ? BASE1 : BASE0;
    wire [28:0] rbase = wbuf ? BASE0 : BASE1;

    // ---------------------------------------------------------------- capture -> write FIFO
    reg  [23:0] even_px;
    wire [7:0]  wx  = hcount[8:1];                                   // word in line, 0..143
    wire [15:0] woff = {vcount[7:0], 7'd0} + {vcount[7:0], 4'd0} + {8'd0, wx};   // y * 144 + x / 2
    (* ramstyle = "MLAB, no_rw_check" *) reg [63:0] f_data [0:63];
    (* ramstyle = "MLAB, no_rw_check" *) reg [28:0] f_addr [0:63];
    reg  [5:0]  f_wp = '0, f_rp = '0;
    reg  [6:0]  f_cnt = '0;
    wire        f_push = flipping && ce_pix && de && hcount[0];
    wire        f_pop;
    always @(posedge clk_sys) begin
        if (ce_pix && de && !hcount[0]) even_px <= rgb_in;
        if (f_push) begin
            if (f_cnt != 7'd64) begin
                f_data[f_wp] <= {8'd0, rgb_in, 8'd0, even_px};
                f_addr[f_wp] <= wbase + {13'd0, woff};
                f_wp <= f_wp + 6'd1;
            end else if (fifo_drops != 16'hFFFF) fifo_drops <= fifo_drops + 16'd1;
        end
        f_cnt <= f_cnt + ((f_push && f_cnt != 7'd64) ? 7'd1 : 7'd0) - (f_pop ? 7'd1 : 7'd0);
        if (f_pop) f_rp <= f_rp + 6'd1;
    end

    // ---------------------------------------------------------------- line buffers (display side)
    // bank = display line parity; fetched during the previous display line
    (* ramstyle = "MLAB, no_rw_check" *) reg [47:0] lbuf [0:287];   // {bank, word 0..143}
    reg  [8:0]  lb_waddr;
    reg  [47:0] lb_wdata;
    reg         lb_we = 1'b0;
    always @(posedge clk_sys) if (lb_we) lbuf[lb_waddr] <= lb_wdata;

    // which display line is fetched next, and whether it made it
    reg  [8:0]  fetch_line = '0;       // display line being fetched (its data goes to bank fetch_line[0])
    reg  [1:0]  bank_ok = 2'b00;       // bank complete for the line it holds
    wire [8:0]  next_line = (vcount == 9'd263) ? 9'd0 : vcount + 9'd1;
    wire [8:0]  fl_next   = (next_line == 9'd263) ? 9'd0 : next_line + 9'd1;   // line fetched now
    wire [7:0]  fl_row    = 8'd223 - fl_next[7:0];                            // its stored line

    // ---------------------------------------------------------------- DDR3 master
    localparam [1:0] M_IDLE = 2'd0, M_WR = 2'd1, M_RD = 2'd2, M_BEATS = 2'd3;
    reg [1:0]  m = M_IDLE;
    reg        rd_pending = 1'b0;
    reg        half = 1'b0;            // burst 0 = words 0..71, burst 1 = 72..143
    reg [7:0]  beats = '0;
    reg [28:0] m_addr = '0;
    reg [63:0] m_din = '0;
    reg        m_we = 1'b0, m_rd = 1'b0;
    reg [28:0] line_base = '0;
    assign ddr_burstcnt = m_rd ? 8'd72 : 8'd1;
    assign ddr_addr     = m_addr;
    assign ddr_din      = m_din;
    assign ddr_be       = 8'hFF;
    assign ddr_we       = m_we;
    assign ddr_rd       = m_rd;
    assign f_pop        = (m == M_IDLE) && own && !rd_pending && (f_cnt != 7'd0);
    assign ddr_active   = flipping || (m != M_IDLE);

    always @(posedge clk_sys) begin
        lb_we <= 1'b0;
        // a new line starts: fetch the line after it (stored line 223 - that line)
        if (line_end) begin
            if (rd_pending && late_lines != 16'hFFFF) late_lines <= late_lines + 16'd1;
            bank_ok[next_line[0] ^ 1'b1] <= 1'b0;
            // display line L = next_line + 1 (0..223; 0 after line 263) shows stored line 223 - L
            if (flipping && rd_valid && (fl_next < 9'd224)) begin
                fetch_line <= fl_next;
                line_base  <= rbase + {12'd0, fl_row, 7'd0} + {15'd0, fl_row, 4'd0};   // (223 - L) * 144
                rd_pending <= 1'b1;
                half <= 1'b0;
            end
        end
        case (m)
            M_IDLE: if (own) begin
                if (rd_pending) begin
                    m_rd <= 1'b1; m_addr <= line_base + (half ? 29'd72 : 29'd0);
                    m <= M_RD;
                end else if (f_cnt != 7'd0) begin
                    m_we <= 1'b1; m_addr <= f_addr[f_rp]; m_din <= f_data[f_rp];
                    m <= M_WR;
                end
            end
            M_WR: if (!ddr_busy) begin m_we <= 1'b0; m <= M_IDLE; end
            M_RD: if (!ddr_busy) begin m_rd <= 1'b0; beats <= 8'd0; m <= M_BEATS; end
            M_BEATS: if (ddr_dout_ready) begin
                lb_we    <= 1'b1;
                lb_waddr <= (fetch_line[0] ? 9'd144 : 9'd0) + (half ? 9'd72 : 9'd0) + {1'b0, beats};
                lb_wdata <= {ddr_dout[55:32], ddr_dout[23:0]};
                beats <= beats + 8'd1;
                if (beats == 8'd71) begin
                    if (half) begin rd_pending <= 1'b0; bank_ok[fetch_line[0]] <= 1'b1; end
                    half <= ~half;
                    m <= M_IDLE;
                end
            end
        endcase
    end

    // ---------------------------------------------------------------- display: (x, y) <- (287 - x, 223 - y)
    wire [8:0]  mx    = 9'd287 - hcount;
    reg  [47:0] rd_w;
    reg         rd_hi, rd_show;
    always @(posedge clk_sys) begin
        rd_w    <= lbuf[(vcount[0] ? 9'd144 : 9'd0) + {1'b0, mx[8:1]}];
        rd_hi   <= mx[0];
        rd_show <= flipping && rd_valid && de && bank_ok[vcount[0]];
        rgb_out <= !rd_show ? 24'd0 : rd_hi ? rd_w[47:24] : rd_w[23:0];
    end
endmodule
