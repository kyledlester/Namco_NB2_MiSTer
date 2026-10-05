// Namco NB-1 MiSTer core -- C123 tilemap line renderer (M10).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Renders the six C123 tile layers of one scanline into a line buffer while
// the previous scanline is displayed (one line of lookahead, like the PCB's
// tile pipeline). Behaviour is MAME 0.289's [MAME-SOURCE namco_c123tmap.cpp,
// tilemap.cpp, namconb1.cpp screen_update]; docs/M10_RESEARCH.md derives
// every formula below and scripts/research/nb1_m10_ref.py restates it (that
// model equals MAME pixel for pixel on the self-test frames).
//
//   layers 0-3  64x64 tiles, VRAM words L*$1000 + row*64 + col,
//               x = (sx + 44 + {4,2,1,0}[L] + X) mod 512, y = (sy + 24 + Y) mod 512
//   layers 4/5  36x28 tiles, VRAM words $4008/$4408 + row*36 + col, x = X, y = Y
//   flip        (ctl w1 bit 15, all layers) memory x = 511-x (287-X), y = 511-y (223-Y)
//   pixel       CHR[code*64 + (y&7)*8 + (x&7)], 8bpp, one 8-byte SDRAM line per tile row
//   opaque      SHAPE[code*8 + (y&7)] bit 7-(x&7); the 8-byte SHAPE line is the whole tile mask
//   pen         $1000 + bank*256 + pixel, bank = ctl w(24+L) & 7
//   enable      !(ctl w(16+L) bit 3);  order = priority 0..7, then layer 0..5, later opaque wins
//
// MAME quirk not copied [IMPLEMENTATION-DECISION]: MAME negates a scroll value
// at the moment it is written if the flip bit is set at that moment, so a
// scroll register written before a flip change keeps the old sign. Here the
// stored register and the current flip bit are combined when the line is
// drawn. Nebulas Ray rewrites the scroll registers after every flip change it
// makes, so both give the same picture (sim: self-test frames equal).
//
// Pipeline (all in clk_sys, one line = 384 x 16 = 6,144 clk_sys of budget):
//   walker   per enabled layer in draw order, per tile of the line: read the
//            tile code (VRAM video port), look the code up in the SHAPE cache
//            (512 x 64-bit direct-mapped, 4 M10K) or fetch its 8-byte SHAPE
//            line; tiles whose mask row is 0 are skipped (no CHR fetch).
//   fetcher  one 8-byte CHR fetch per remaining tile row (region CHR).
//   writer   8 pixels per tile row into the line buffer, 1 per clk_sys,
//            opaque and on-screen pixels only (painter's order = MAME order).
// Job FIFO (4) between walker and fetcher; a single nb1_memory client port
// shared by walker (SHAPE, first) and fetcher. M23: up to two reads
// outstanding (nb1_memory PIPE); a tag queue routes the in-order responses
// (SHAPE to the walker, CHR into a 2-entry completed-row queue that feeds the
// writer), so consecutive CHR reads follow each other at the controller's pace.
//
// Line buffer: 2 x 512 x 16, one simple dual-port M10K array (2 blocks):
// the read port belongs to the display; the write port to the writer, except
// that the display clears each entry the clk_sys after reading it (so every
// displayed line starts empty). That clear takes the write port 1 clk_sys in
// 16 and the writer pauses for it. Entry: [15] valid, [14] 0, [13:11] layer priority,
// [10:8] palette bank, [7:0] pixel. The priority is kept for the later
// sprite mixer; M10 uses only valid/bank/pixel.
//
// Deadline: rendering of line T starts at the line_end that begins the
// display of line T-1 and must end by the next line_end. A line still being
// drawn then is abandoned (underrun counted; its buffer is shown as drawn so
// far, never stale data) and an outstanding SDRAM read is drained and
// discarded. Lines 224-263 are not drawn; line 0 is drawn during line 263.
module nb1_c123_render (
    input  wire         clk_sys,
    input  wire         reset,          // idle + SHAPE-cache flush (512 clk after release)

    // raster (nb1_video_timing)
    input  wire         line_end,
    input  wire [8:0]   vcount,

    // C123 state
    input  wire [511:0] ctl,            // nb1_c123_ctl regs_out
    output reg  [14:0]  vram_addr = '0, // VRAM video port (word), data 1 clk later
    input  wire [15:0]  vram_data,

    // nb1_memory client (docs/M2_IMPLEMENTATION.md section 4), reads only
    output reg          mreq_valid = 1'b0,
    input  wire         mreq_ready,
    output reg  [3:0]   mreq_region = '0,
    output reg  [24:0]  mreq_offset = '0,
    output wire         mreq_we,
    output wire [31:0]  mreq_wdata,
    output wire [3:0]   mreq_be,
    input  wire         mrsp_valid,
    input  wire [63:0]  mrsp_line,
    input  wire         mrsp_err,

    // display port (nb1_video_out)
    input  wire         disp_rd,        // read entry disp_x of buffer disp_buf, clear it next clk
    input  wire         disp_buf,
    input  wire [8:0]   disp_x,
    output wire [15:0]  disp_entry,     // valid the clk after disp_rd (capture it then)

    // diagnostics (per-frame values are latched when line 0 starts)
    output reg  [5:0]   layer_mask = '0,    // enabled layers of the last started line
    output reg          flip_out = 1'b0,
    output reg  [15:0]  chr_frame = '0,     // CHR fetches in the last frame
    output reg  [15:0]  shape_frame = '0,   // SHAPE fetches in the last frame
    output reg  [15:0]  worst_frame = '0,   // longest line render time in the last frame (clk_sys)
    output reg  [15:0]  underruns = '0,     // lines abandoned at their deadline (saturating)
    output reg  [31:0]  lines_done = '0,    // lines completed in time
    output wire         line_busy,          // a line is being drawn and is not complete yet (M11: sprite yield)
    output reg          mem_error = 1'b0    // an SDRAM read returned rsp_err
);
    import nb1_mem_pkg::*;

    assign mreq_we    = 1'b0;
    assign mreq_wdata = '0;
    assign mreq_be    = 4'b1111;

    reg [15:0] chr_cnt = '0, shape_cnt = '0, worst_cnt = '0;

    // ------------------------------------------------------ line start / abort
    wire [8:0] disp_next = (vcount == 9'd263) ? 9'd0 : vcount + 9'd1;   // line whose display starts
    wire [8:0] tgt_next  = (disp_next == 9'd263) ? 9'd0 : disp_next + 9'd1;
    wire       start_now = (tgt_next < 9'd224);   // qualified by line_end below

    // walker states
    localparam [3:0] W_IDLE = 4'd0, W_CLEAR = 4'd1, W_SCAN = 4'd2, W_SETUP = 4'd3, W_TILE = 4'd4,
                     W_VWAIT = 4'd5, W_CODE = 4'd6, W_SLOOK = 4'd8, W_SFILL = 4'd9,
                     W_PUSH = 4'd10, W_DONE = 4'd11;
    reg [3:0]  wst = W_CLEAR;
    reg [8:0]  clr_idx = '0;

    // per-line latched state
    reg        line_active = 1'b0;   // a line is being drawn
    reg [7:0]  ln_y = '0;            // target line T (0..223)
    reg        ln_buf = 1'b0;
    reg        ln_flip = 1'b0;
    reg [8:0]  ln_sx [0:3];
    reg [8:0]  ln_sy [0:3];
    reg [2:0]  ln_pri [0:5];
    reg [2:0]  ln_bank [0:5];
    reg [5:0]  ln_en = '0;
    reg [15:0] ln_cycles = '0;

    // layer scan
    reg [3:0]  sc_pri = '0;          // 8 = every priority scanned
    reg [2:0]  sc_l = '0;
    reg [2:0]  cur_l = '0;

    // current layer parameters
    reg [5:0]  lp_col0 = '0;         // first tile column (scroll layers, before flip)
    reg [2:0]  lp_o = '0;            // pixels of the first tile left of X = 0
    reg [5:0]  lp_row = '0;
    reg [2:0]  lp_fine = '0;
    reg [14:0] lp_fbase = '0;        // fixed layers: $4008/$4408 + row*36
    reg        lp_fixed = 1'b0;
    reg [5:0]  lp_ntiles = '0;
    reg [2:0]  lp_pri = '0, lp_bank = '0;

    reg [5:0]  tk = '0;              // tile index along the line
    reg signed [9:0] t_bx = '0;      // screen X of the tile's leftmost column
    reg [15:0] t_code = '0;
    reg [7:0]  t_mask = '0;

    // ------------------------------------------------------------ SHAPE cache
    // entry = {valid, tag = code[15:9], 8 mask bytes (row 0 in [63:56])}
    (* ramstyle = "M10K, no_rw_check" *) reg [71:0] sc_mem [0:511];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 512; i++) sc_mem[i] = '0;
`endif
// synthesis translate_on
    // Read address: the tile code straight from the VRAM port in W_CODE (so
    // the entry is ready in W_SLOOK), else the held code of the current tile.
    reg [8:0]  sc_raddr = '0;
    reg [71:0] sc_q = '0;
    reg        sc_we = 1'b0;
    reg [8:0]  sc_waddr = '0;
    reg [71:0] sc_wdata = '0;
    wire [8:0] sc_ra;
    always @(posedge clk_sys) begin
        if (sc_we) sc_mem[sc_waddr] <= sc_wdata;
        sc_q <= sc_mem[sc_ra];
    end
    wire sc_hit = sc_q[71] && (sc_q[70:64] == t_code[15:9]);

    function automatic [7:0] row_of(input [63:0] l, input [2:0] r);
        row_of = l[63 - 8 * r -: 8];
    endfunction
    wire [7:0] hit_mask = row_of(sc_q[63:0], lp_fine);

    // --------------------------------------------------------------- job FIFO
    // {bx[9:0], mask[7:0], pri[2:0], bank[2:0], code[15:0], fine[2:0]} = 43 bits
    (* ramstyle = "logic" *) reg [42:0] fifo [0:3];   // M16: tiny; kept out of M10K
    reg [1:0]  f_rd = '0, f_wr = '0;
    reg [2:0]  f_cnt = '0;
    wire       f_full  = (f_cnt == 3'd4);
    wire       f_empty = (f_cnt == 3'd0);
    wire [42:0] f_head = fifo[f_rd];
    wire       f_push;
    wire       f_pop;

    // --------------------------------------------------------- memory port
    // M23: reads in flight, in issue order: {CHR (1) / SHAPE (0), CHR job}
    reg [43:0] tg [0:1];
    reg        tg_rd = 1'b0, tg_wr = 1'b0;
    reg [1:0]  t_out = '0;          // accepted by nb1_memory, not yet answered
    reg [1:0]  t_chr = '0;          // CHR reads posted or accepted, not yet answered (and not dropped)
    reg [1:0]  t_disc = '0;         // oldest responses to drop (abandoned line)
    wire       mem_pending = mreq_valid || (t_out != 2'd0);
    wire       rsp_mine = (t_out != 2'd0) && mrsp_valid;
    wire       rsp_use  = rsp_mine && (t_disc == 2'd0);
    wire [43:0] tg_head = tg[tg_rd];
    wire       rsp_chr  = rsp_use && tg_head[43];
    wire       rsp_shp  = rsp_use && !tg_head[43];
    // room for one more read at nb1_memory: nothing posted, fewer than two accepted after this cycle
    wire       port_room = !mreq_valid && ((t_out - (rsp_mine ? 2'd1 : 2'd0)) < 2'd2);
    wire       walker_wants = (wst == W_SLOOK) && !sc_hit;

    // ------------------------------------------------------------- fetcher
    // The job leaves the FIFO when its CHR read is issued (it then travels in the
    // tag queue); the answered row waits in the completed-row queue for the writer.
    // M23: a CHR read is issued only if its row will have a place: answered-and-
    // waiting rows + CHR reads in flight < 2.
    reg [42:0] dq_job [0:1];
    reg [63:0] dq_data [0:1];
    reg        dq_rd = 1'b0, dq_wr = 1'b0;
    reg [1:0]  dq_cnt = '0;
    wire [42:0] fj = dq_job[dq_rd];

    // -------------------------------------------------------------- writer
    reg        wr_busy = 1'b0;
    reg [2:0]  wr_i = '0;
    reg signed [9:0] wr_bx = '0;
    reg [7:0]  wr_mask = '0;
    reg [63:0] wr_data = '0;
    reg [2:0]  wr_pri = '0, wr_bank = '0;
    reg        wr_flip = 1'b0;
    reg        wr_buf = 1'b0;

    // ---------------------------------------------------------- line buffer
    reg        dclr = 1'b0;          // clear the entry the display read last clk_sys
    reg [9:0]  dclr_addr = '0;
    always @(posedge clk_sys) begin
        dclr      <= disp_rd;
        dclr_addr <= {disp_buf, disp_x};
    end

    wire signed [9:0] wr_x = wr_flip ? (wr_bx + 10'sd7 - $signed({7'd0, wr_i})) : (wr_bx + $signed({7'd0, wr_i}));
    wire       wr_on  = wr_busy && !dclr && wr_mask[3'd7 - wr_i] && (wr_x >= 10'sd0) && (wr_x < 10'sd288);
    wire [7:0] wr_pix = wr_data[63 - 8 * wr_i -: 8];

    (* ramstyle = "M10K, no_rw_check" *) reg [15:0] lb_mem [0:1023];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 1024; i++) lb_mem[i] = '0;
`endif
// synthesis translate_on
    reg [15:0] lb_q = '0;
    wire       lb_we    = dclr || wr_on;
    wire [9:0] lb_waddr = dclr ? dclr_addr : {wr_buf, wr_x[8:0]};
    wire [15:0] lb_wdata = dclr ? 16'h0000 : {1'b1, 1'b0, wr_pri, wr_bank, wr_pix};
    always @(posedge clk_sys) begin
        if (lb_we) lb_mem[lb_waddr] <= lb_wdata;
        if (disp_rd) lb_q <= lb_mem[{disp_buf, disp_x}];
    end
    assign disp_entry = lb_q;

    // ----------------------------------------------------------- completion
    wire walker_done = (wst == W_DONE);
    wire line_done   = line_active && walker_done && f_empty && (dq_cnt == 2'd0) && !wr_busy && !mem_pending;
    wire busy_now    = line_active && !line_done;
    assign line_busy = busy_now;

    // --------------------------------------------------------- tile address
    wire [5:0]  tk_c6   = lp_col0 + tk;
    wire [5:0]  tk_col  = lp_fixed ? (ln_flip ? 6'd35 - tk : tk) : (ln_flip ? ~tk_c6 : tk_c6);
    wire [14:0] tk_vaddr = lp_fixed ? (lp_fbase + {9'd0, tk_col}) : {1'b0, cur_l[1:0], lp_row, tk_col};
    wire signed [9:0] tk_bx = $signed({1'b0, tk, 3'b000}) - $signed({7'd0, lp_o});

    // layer parameters for layer cur_l of line ln_y
    wire [9:0] s_sx   = {1'b0, ln_sx[cur_l[1:0]]} + (cur_l == 3'd0 ? 10'd48 : cur_l == 3'd1 ? 10'd46 :
                                                    cur_l == 3'd2 ? 10'd45 : 10'd44);
    wire [9:0] s_sy   = {1'b0, ln_sy[cur_l[1:0]]} + 10'd24 + {2'd0, ln_y};
    wire [8:0] s_y    = ln_flip ? ~s_sy[8:0] : s_sy[8:0];
    wire [7:0] f_y    = ln_flip ? 8'd223 - ln_y : ln_y;
    wire [14:0] f_base = 15'h4008 + (cur_l == 3'd5 ? 15'h0400 : 15'h0000) +
                         {5'd0, f_y[7:3], 5'd0} + {8'd0, f_y[7:3], 2'd0};

    // push: a cache hit with a non-empty row (W_SLOOK) or a filled row (W_PUSH)
    wire   push_hit  = (wst == W_SLOOK) && sc_hit && (hit_mask != 8'd0);
    wire   push_fill = (wst == W_PUSH) && (t_mask != 8'd0);
    wire [7:0] push_mask = push_hit ? hit_mask : t_mask;
    assign f_push = (push_hit || push_fill) && !f_full && line_active;
    wire   t_abandon = line_end && wst != W_CLEAR && busy_now;   // M23: this line_end abandons the line
    wire   f_issue = !f_empty && port_room && ({1'b0, dq_cnt} + {1'b0, t_chr} < 3'd2) && !walker_wants && line_active && !t_abandon;
    assign f_pop  = f_issue;
    assign sc_ra  = (wst == W_CODE) ? vram_data[8:0] : sc_raddr;
    // writer load: the oldest answered row
    wire   wr_load = !wr_busy && (dq_cnt != 2'd0);
    wire [63:0] wr_load_data = dq_data[dq_rd];

    integer n;
    always @(posedge clk_sys) begin
        sc_we <= 1'b0;

        // ---------------------------------------------------- memory handshake
        if (mreq_valid && mreq_ready) mreq_valid <= 1'b0;
        t_out <= t_out + ((mreq_valid && mreq_ready) ? 2'd1 : 2'd0) - (rsp_mine ? 2'd1 : 2'd0);
        if (rsp_mine && t_disc != 2'd0) t_disc <= t_disc - 2'd1;
        if (rsp_use) begin
            tg_rd <= !tg_rd;
            if (mrsp_err) mem_error <= 1'b1;
        end
        // completed-row queue: CHR responses in, writer loads out
        if (rsp_chr) begin
            dq_job[dq_wr]  <= tg_head[42:0];
            dq_data[dq_wr] <= mrsp_err ? 64'd0 : mrsp_line;
            dq_wr <= !dq_wr;
        end
        if (wr_load) dq_rd <= !dq_rd;
        dq_cnt <= dq_cnt + (rsp_chr ? 2'd1 : 2'd0) - (wr_load ? 2'd1 : 2'd0);
        t_chr  <= t_chr + (f_issue ? 2'd1 : 2'd0) - (rsp_chr ? 2'd1 : 2'd0);

        // ------------------------------------------------------------- FIFO
        if (f_push) begin
            fifo[f_wr] <= {t_bx, push_mask, lp_pri, lp_bank, t_code, lp_fine};
            f_wr <= f_wr + 2'd1;
        end
        if (f_pop) f_rd <= f_rd + 2'd1;
        f_cnt <= f_cnt + (f_push ? 3'd1 : 3'd0) - (f_pop ? 3'd1 : 3'd0);

        // ------------------------------------------------------------ walker
        case (wst)
            W_CLEAR: begin
                sc_we    <= 1'b1;
                sc_waddr <= clr_idx;
                sc_wdata <= '0;
                clr_idx  <= clr_idx + 9'd1;
                if (clr_idx == 9'd511) wst <= W_IDLE;
            end
            W_SCAN: begin
                // MAME order: priority 0..7, then layer 0..5 (one candidate per clk)
                if (sc_pri[3]) wst <= W_DONE;
                else begin
                    if (ln_en[sc_l] && ln_pri[sc_l] == sc_pri[2:0]) begin
                        cur_l <= sc_l;
                        wst   <= W_SETUP;
                    end
                    if (sc_l == 3'd5) begin
                        sc_l   <= 3'd0;
                        sc_pri <= sc_pri + 4'd1;
                    end else begin
                        sc_l <= sc_l + 3'd1;
                    end
                end
            end
            W_SETUP: begin
                lp_fixed  <= cur_l[2];
                lp_ntiles <= cur_l[2] ? 6'd36 : 6'd37;
                lp_o      <= cur_l[2] ? 3'd0 : s_sx[2:0];
                lp_col0   <= s_sx[8:3];
                lp_row    <= s_y[8:3];
                lp_fine   <= cur_l[2] ? f_y[2:0] : s_y[2:0];
                lp_fbase  <= f_base;
                lp_pri    <= ln_pri[cur_l];
                lp_bank   <= ln_bank[cur_l];
                tk        <= 6'd0;
                wst       <= W_TILE;
            end
            W_TILE: begin
                if (tk == lp_ntiles) wst <= W_SCAN;
                else if (tk_bx >= 10'sd288) tk <= tk + 6'd1;
                else begin
                    vram_addr <= tk_vaddr;
                    t_bx      <= tk_bx;
                    wst       <= W_VWAIT;
                end
            end
            W_VWAIT: wst <= W_CODE;
            W_CODE: begin
                t_code   <= vram_data;
                sc_raddr <= vram_data[8:0];
                wst      <= W_SLOOK;
            end
            W_SLOOK: begin
                if (sc_hit) begin
                    // empty row: skip; else push now (or wait for FIFO space)
                    if (hit_mask == 8'd0 || !f_full) begin
                        tk  <= tk + 6'd1;
                        wst <= W_TILE;
                    end
                end else if (port_room && !t_abandon) begin
                    mreq_valid  <= 1'b1;
                    mreq_region <= REG_SHAPE;
                    mreq_offset <= {6'd0, t_code, 3'b000};
                    tg[tg_wr]   <= {1'b0, 43'd0};
                    tg_wr       <= !tg_wr;
                    shape_cnt   <= shape_cnt + 16'd1;
                    wst         <= W_SFILL;
                end
            end
            W_SFILL: if (rsp_shp) begin
                sc_we    <= 1'b1;
                sc_waddr <= t_code[8:0];
                sc_wdata <= {!mrsp_err, t_code[15:9], mrsp_line};
                t_mask   <= mrsp_err ? 8'd0 : row_of(mrsp_line, lp_fine);
                wst      <= W_PUSH;
            end
            W_PUSH: begin
                if (t_mask == 8'd0 || !f_full) begin
                    tk  <= tk + 6'd1;
                    wst <= W_TILE;
                end
            end
            default: ;   // W_IDLE, W_DONE: wait for a line start
        endcase

        // ----------------------------------------------------------- fetcher
        if (f_issue) begin
            mreq_valid  <= 1'b1;
            mreq_region <= REG_CHR;
            mreq_offset <= {3'd0, f_head[18:3], f_head[2:0], 3'b000};
            tg[tg_wr]   <= {1'b1, f_head};
            tg_wr       <= !tg_wr;
            chr_cnt     <= chr_cnt + 16'd1;
        end

        // ------------------------------------------------------------ writer
        if (wr_load) begin
            wr_busy <= 1'b1;
            wr_i    <= 3'd0;
            wr_bx   <= $signed(fj[42:33]);
            wr_mask <= fj[32:25];
            wr_pri  <= fj[24:22];
            wr_bank <= fj[21:19];
            wr_data <= wr_load_data;
            wr_flip <= ln_flip;
            wr_buf  <= ln_buf;
        end else if (wr_busy && !dclr) begin      // paused while the display clears
            wr_i <= wr_i + 3'd1;
            if (wr_i == 3'd7) wr_busy <= 1'b0;
        end

        // ------------------------------------------------- line bookkeeping
        if (line_active) ln_cycles <= (ln_cycles == 16'hFFFF) ? ln_cycles : ln_cycles + 16'd1;
        if (line_done) begin
            line_active <= 1'b0;
            lines_done  <= lines_done + 32'd1;
            if (ln_cycles > worst_cnt) worst_cnt <= ln_cycles;
        end

        // Every line_end is a deadline: the line being drawn must be complete
        // before its display starts (the last visible line included).
        if (line_end && wst != W_CLEAR && busy_now) begin
            if (underruns != 16'hFFFF) underruns <= underruns + 16'd1;
            worst_cnt   <= 16'hFFFF;
            // M23: every read still owed to this line (posted or accepted) is dropped on return
            t_disc <= t_out + (mreq_valid ? 2'd1 : 2'd0) - (rsp_mine ? 2'd1 : 2'd0);
            tg_rd <= 1'b0; tg_wr <= 1'b0; t_chr <= '0;
            dq_rd <= 1'b0; dq_wr <= 1'b0; dq_cnt <= '0;
            f_rd <= '0; f_wr <= '0; f_cnt <= '0;
            wr_busy     <= 1'b0;
            line_active <= 1'b0;
            wst         <= W_IDLE;
        end
        if (line_end && start_now && wst != W_CLEAR) begin
            // latch the C123 state for line tgt_next
            line_active <= 1'b1;
            ln_cycles   <= 16'd0;
            ln_y        <= tgt_next[7:0];
            ln_buf      <= tgt_next[0];
            ln_flip     <= ctl[1*16 + 15];
            for (n = 0; n < 4; n++) begin
                ln_sx[n] <= ctl[(4*n + 1)*16 +: 9];
                ln_sy[n] <= ctl[(4*n + 3)*16 +: 9];
            end
            for (n = 0; n < 6; n++) begin
                ln_pri[n]  <= ctl[(16 + n)*16 +: 3];
                ln_bank[n] <= ctl[(24 + n)*16 +: 3];
                ln_en[n]   <= !ctl[(16 + n)*16 + 3];
            end
            for (n = 0; n < 6; n++) layer_mask[n] <= !ctl[(16 + n)*16 + 3];
            flip_out <= ctl[1*16 + 15];
            sc_pri <= 4'd0;
            sc_l   <= 3'd0;
            wst    <= W_SCAN;
            if (tgt_next == 9'd0) begin
                chr_frame   <= chr_cnt;
                shape_frame <= shape_cnt;
                worst_frame <= worst_cnt;
                chr_cnt     <= 16'd0;
                shape_cnt   <= 16'd0;
                worst_cnt   <= 16'd0;
            end
        end

        if (reset) begin
            wst         <= W_CLEAR;
            clr_idx     <= '0;
            line_active <= 1'b0;
            mreq_valid  <= 1'b0;
            t_out <= '0; t_disc <= '0; t_chr <= '0; tg_rd <= 1'b0; tg_wr <= 1'b0;
            dq_rd <= 1'b0; dq_wr <= 1'b0; dq_cnt <= '0;
            f_rd <= '0; f_wr <= '0; f_cnt <= '0;
            wr_busy     <= 1'b0;
            chr_cnt     <= '0;
            shape_cnt   <= '0;
            worst_cnt   <= '0;
            chr_frame   <= '0;
            shape_frame <= '0;
            worst_frame <= '0;
            underruns   <= '0;
            lines_done  <= '0;
            mem_error   <= 1'b0;
            layer_mask  <= '0;
            flip_out    <= 1'b0;
        end
    end
endmodule
