// Namco NB-2 MiSTer core -- C169 ROZ (rotate/zoom) line renderer, two layers.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Behaviour = MAME 0.289 namco_c169roz (is_namcofl = false) with the NB-2 ROZ tile callbacks
// [MAME-SOURCE namco_c169roz.cpp, namconb1.cpp 502-518]; scripts/research/nb2_video_model.py restates it and equals
// MAME pixel for pixel on 26 captured frames (docs/MILESTONES.md). Per screen pixel (x, y) of layer L:
//   parameters  words 8L..8L+7 of the control registers, or (L = 1 and control word 0 == $8000) the 8 words at
//               VRAM byte $E080 + (y >> 3) * $100 + (y & 7) * $10 (per-line mode); word 1 bit 15 disables.
//   unpack      size = 512 << w1[9:8], colour = w1[3:0], priority = w1[7:4]; inc = 13-bit signed {w[15], w[11:0]}
//               (incxx = w2, incxy = w3, incyx = w4, incyy = w5); left = w2[14:12] * 512, top = w3[14:12] * 512.
//   position    X = (s16(w6) << 4) + (36 + x) * incxx + (3 + y) * incyx, Y likewise with w7/incxy/incyy (only
//               X[19:8] matters: MAME's 32-bit (X << 8) >> 16 masked by size-1 <= $FFF);
//               xpos = ((X[19:8] & (size-1)) + left) & $FFF, ypos likewise.
//   tile        VRAM[((xpos >> 4) & $80) << 8 | (ypos >> 4) << 7 | (xpos >> 4) & $7F] & $3FFF = code;
//               mangle = (code & $7FF) | rozbank[8L + code[13:11]] << 11;
//               mask tile = mangle; pixel tile = xform ? mangle with bits 4 and 6 exchanged : mangle.
//   pixel       ROZ[tile * 256 + (ypos & 15) * 16 + (xpos & 15)], drawn iff
//               ROZMASK[mask * 32 + (ypos & 15) * 2 + (xpos & 15) / 8] bit 7 - (xpos & 7) is set;
//               pen = $1800 + colour * 256 + pixel (the mixer adds $1800).
// The pixel tile wraps in the 8 MiB ROZ window (15 bits), the mask tile in the 2 MiB mask window (16 bits); MAME
// wraps the pixel tile modulo the tile count instead, which differs only for banks beyond the ROMs.
//
// Pipeline (clk_sys; one line = 6,144 clocks): a walker (4 clocks per pixel: position + VRAM address, VRAM wait,
// tile code + callback + cache index, cache data) pushes one job per pixel; for each job it queues at most two 8-byte reads
// in job order: the mask word (4 tile rows) unless the 512-entry mask cache holds it or it is the mask word
// requested last, and the pixel word unless it is the word requested last or the cached mask bit says the pixel
// is transparent. Reads go out in order (two outstanding, nb2_memory), answers come back in order, and the writer
// consumes them job by job, fills the mask cache, and writes opaque pixels into its layer's line buffer.
// Line timing and the abandon-at-deadline rule are the C123 renderer's (nb1_c123_render): line T is drawn while
// line T-1 is shown, from the state latched at that line_end.
// Line buffers: per layer 2 x 512 x 17 ({opaque, priority[3:0], colour[3:0], pixel[7:0]}); the display port reads
// both layers and clears the entry the clock after.
module nb2_roz_render (
    input  wire         clk_sys,
    input  wire         reset,

    input  wire         line_end,
    input  wire [8:0]   vcount,
    input  wire         oflip,          // OSD 180 degrees: display line T shows logical line 223 - T

    input  wire [255:0] ctl,              // C169 control words {w15 .. w0}
    input  wire [127:0] rbank,            // ROZ bank bytes $980000-$98000F, byte 0 in [127:120]
    input  wire         swap46,           // board record xform bit 2
    output reg  [15:0]  vram_addr = '0,   // ROZ VRAM port (word), data one clock later
    input  wire [15:0]  vram_data,

    output wire         mreq_valid,
    input  wire         mreq_ready,
    output wire [3:0]   mreq_region,
    output wire [25:0]  mreq_offset,
    output wire         mreq_we,
    output wire [31:0]  mreq_wdata,
    output wire [3:0]   mreq_be,
    input  wire         mrsp_valid,
    input  wire [63:0]  mrsp_line,
    input  wire         mrsp_err,

    input  wire         disp_rd,
    input  wire         disp_buf,
    input  wire [8:0]   disp_x,
    output wire [16:0]  disp_e0,          // layer 0 entry, valid the clock after disp_rd
    output wire [16:0]  disp_e1,

    output reg  [15:0]  reads_frame = '0,
    output reg  [15:0]  worst_frame = '0,
    output reg  [15:0]  underruns = '0,
    output reg          mem_error = 1'b0
);
    import nb2_mem_pkg::*;
    assign mreq_we = 1'b0;
    assign mreq_wdata = '0;
    assign mreq_be = 4'hF;

    function automatic [15:0] cw(input integer i);
        cw = ctl[16*i +: 16];
    endfunction
    function automatic signed [19:0] s13(input [15:0] v);      // MAME: sign from bit 15, value bits 11-0
        s13 = {{8{v[15]}}, v[11:0]};
    endfunction

    // ------------------------------------------------------------------ line start / abort (C123 timing)
    wire [8:0] disp_next = (vcount == 9'd263) ? 9'd0 : vcount + 9'd1;
    wire [8:0] tgt_next  = (disp_next == 9'd263) ? 9'd0 : disp_next + 9'd1;
    wire       start_now = (tgt_next < 9'd224);

    localparam [3:0] W_IDLE = 4'd0, W_CLEAR = 4'd1, W_LAYER = 4'd2, W_PRM = 4'd3, W_PRMW = 4'd4, W_SETUP = 4'd5,
                     W_MUL = 4'd6, W_A = 4'd7, W_B = 4'd8, W_C = 4'd9, W_DONE = 4'd10, W_V = 4'd11;
    reg [3:0]  wst = W_CLEAR;
    reg [8:0]  clr_idx = '0;
    reg        line_active = 1'b0;
    reg [7:0]  ln_y = '0;
    reg        ln_buf = 1'b0;
    reg [255:0] ln_ctl = '0;
    reg [127:0] ln_bank = '0;
    reg        ln_swap = 1'b0;
    reg [15:0] ln_cycles = '0, worst_cnt = '0, reads_cnt = '0;

    // per-layer parameters
    reg        lay = 1'b0;                 // layer being walked
    reg [15:0] pw [0:7];                   // its 8 parameter words
    reg [2:0]  prm_k = '0;
    reg        lay_scan = 1'b0;
    reg [11:0] sm = '0;                    // size - 1
    reg [11:0] p_left = '0, p_top = '0;
    reg [3:0]  p_col = '0, p_pri = '0;
    reg signed [19:0] incxx = '0, incxy = '0, incyx = '0, incyy = '0;
    reg [19:0] X = '0, Y = '0;
    reg [8:0]  px = '0;                    // pixel being walked
    // multiply (3 + y) * incyx / incyy once per layer (registered product, inferred multipliers)
    reg signed [19:0] m_x = '0, m_y = '0;
    wire signed [9:0] y3 = $signed({2'b00, ln_y}) + 10'sd3;

    // ------------------------------------------------------------------ walker datapath
    wire [11:0] xpos_c = (X[19:8] & sm) + p_left;
    wire [11:0] ypos_c = (Y[19:8] & sm) + p_top;
    reg  [11:0] xpos = '0, ypos = '0;
    wire [7:0]  tcol = xpos_c[11:4], trow = ypos_c[11:4];
    wire [15:0] vidx = {tcol[7], trow, tcol[6:0]};
    wire [13:0] code = vram_data[13:0];
    wire [7:0]  bnk = ln_bank[127 - 8 * {lay, code[13:11]} -: 8];
    wire [18:0] mangle = {bnk, code[10:0]};
    wire [18:0] ptile = ln_swap ? {mangle[18:7], mangle[4], mangle[5], mangle[6], mangle[3:0]} : mangle;
    wire [15:0] mtile = mangle[15:0];
    // pixel word key / offset: tile[14:0], row, half (8 MiB window); mask word key: mask, row[3:2]
    reg  [19:0] b_pkey = '0;               // {tile[14:0], row[3:0], half}
    reg  [17:0] b_mkey = '0;               // {mask[15:0], row[3:2]}
    reg  [5:0]  b_mbit = '0;               // bit index in the mask word (from the MSB)
    reg  [2:0]  b_pbyte = '0;

    // mask cache: 512 x {valid, tag = mask[15:7], 64-bit mask word}; index {mask[6:0], row[3:2]}
    (* ramstyle = "M10K, no_rw_check" *) reg [73:0] mc [0:511];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 512; i++) mc[i] = '0;
`endif
// synthesis translate_on
    reg  [73:0] mc_q = '0;
    reg         mc_we = 1'b0;
    reg  [8:0]  mc_wa = '0;
    reg  [73:0] mc_wd = '0;
    wire [8:0]  mc_ra = {mangle[6:0], ypos[3:2]};
    always @(posedge clk_sys) begin
        if (mc_we) mc[mc_wa] <= mc_wd;
        mc_q <= mc[mc_ra];
    end
    wire mc_hit = mc_q[73] && (mc_q[72:64] == b_mkey[17:9]);
    wire mc_bit = mc_q[63 - b_mbit];

    // last keys requested in this layer (the writer keeps the matching data)
    reg        lk_m_v = 1'b0, lk_p_v = 1'b0;
    reg [17:0] lk_m = '0;
    reg [19:0] lk_p = '0;

    // ------------------------------------------------------------------ job FIFO (walker -> writer)
    // {x[8:0], layer, pri[3:0], col[3:0], msrc[1:0] (0 known bit, 1 fetched now, 2 last fetched), mbitval,
    //  mbit[5:0], pfetch, pbyte[2:0], mkey[17:0]} = 50 bits
    localparam int JW = 50;
    (* ramstyle = "logic" *) reg [JW-1:0] jq [0:7];
    reg [2:0] jq_rd = '0, jq_wr = '0;
    reg [3:0] jq_cnt = '0;
    wire      jq_full = (jq_cnt >= 4'd7);

    // ------------------------------------------------------------------ read request FIFO (walker -> port)
    // {is_mask, offset[25:0]}
    (* ramstyle = "logic" *) reg [26:0] rq [0:7];
    reg [2:0] rq_rd = '0, rq_wr = '0;
    reg [3:0] rq_cnt = '0;
    wire      rq_room2 = (rq_cnt <= 4'd6);

    // ------------------------------------------------------------------ memory port (two outstanding)
    reg       r_valid = 1'b0;
    reg [25:0] r_off = '0;
    reg        r_mask = 1'b0;
    reg [1:0] outst = '0;
    reg [2:0] drop = '0;                   // answers of an abandoned line still to come
    assign mreq_valid  = r_valid;
    assign mreq_region = r_mask ? REG_ROZMASK : REG_ROZ;
    assign mreq_offset = r_off;
    // answer FIFO (port -> writer)
    (* ramstyle = "logic" *) reg [63:0] aq [0:3];
    reg [1:0] aq_rd = '0, aq_wr = '0;
    reg [2:0] aq_cnt = '0;

    // ------------------------------------------------------------------ writer
    reg [63:0] w_mword = '0, w_pword = '0;
    reg        w_step = 1'b0;              // 0: mask answer (if any), 1: pixel answer (if any)
    wire [JW-1:0] jh = jq[jq_rd];
    wire [8:0]  j_x = jh[49:41];
    wire        j_l = jh[40];
    wire [3:0]  j_pri = jh[39:36], j_col = jh[35:32];
    wire [1:0]  j_msrc = jh[31:30];
    wire        j_mval = jh[29];
    wire [5:0]  j_mbit = jh[28:23];
    wire        j_pf = jh[22];
    wire [2:0]  j_pbyte = jh[21:19];
    wire [17:0] j_mkey = jh[18:1];

    // line buffers
    reg        dclr = 1'b0;
    reg [9:0]  dclr_addr = '0;
    always @(posedge clk_sys) begin
        dclr      <= disp_rd;
        dclr_addr <= {disp_buf, disp_x};
    end
    (* ramstyle = "M10K, no_rw_check" *) reg [16:0] lb0 [0:1023];
    (* ramstyle = "M10K, no_rw_check" *) reg [16:0] lb1 [0:1023];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 1024; i++) begin lb0[i] = '0; lb1[i] = '0; end
`endif
// synthesis translate_on
    reg [16:0] lb0_q = '0, lb1_q = '0;
    reg        wr_v = 1'b0;
    reg        wr_l = 1'b0;
    reg [9:0]  wr_a = '0;
    reg [16:0] wr_d = '0;
    always @(posedge clk_sys) begin
        if (dclr) begin lb0[dclr_addr] <= '0; lb1[dclr_addr] <= '0; end
        else if (wr_v) begin
            if (wr_l) lb1[wr_a] <= wr_d;
            else      lb0[wr_a] <= wr_d;
        end
        if (disp_rd) begin lb0_q <= lb0[{disp_buf, disp_x}]; lb1_q <= lb1[{disp_buf, disp_x}]; end
    end
    assign disp_e0 = lb0_q;
    assign disp_e1 = lb1_q;

    // ------------------------------------------------------------------ completion
    wire walker_done = (wst == W_DONE);
    wire line_done = line_active && walker_done && (jq_cnt == 0) && (rq_cnt == 0) && !r_valid && (outst == 0) &&
                     (aq_cnt == 0) && !wr_v;
    wire busy_now  = line_active && !line_done;
    wire t_abandon = line_end && (wst != W_CLEAR) && busy_now;

    // ------------------------------------------------------------------ main process
    wire jq_push_ok = !jq_full && rq_room2;
    wire j_need_m = (j_msrc == 2'd1);
    wire w_can_m  = !w_step && j_need_m;                          // waiting for the mask answer
    wire w_can_p  = (w_step || !j_need_m) && j_pf;                // waiting for the pixel answer
    // the line-buffer write lands one clock later: keep it off the display's clear slot (dclr = disp_rd + 1)
    wire w_ready  = (jq_cnt != 0) && !disp_rd &&
                    ((w_can_m && aq_cnt != 0) || (w_can_p && aq_cnt != 0) || (!w_can_m && !w_can_p));
    wire [63:0] w_m_now = (w_can_m) ? aq[aq_rd] : w_mword;
    wire [63:0] w_p_now = (w_can_p) ? aq[aq_rd] : w_pword;
    wire        w_op    = (j_msrc == 2'd0) ? j_mval : w_m_now[63 - j_mbit];
    wire [7:0]  w_pix   = w_p_now[63 - 8 * j_pbyte -: 8];
    wire        port_take = r_valid && mreq_ready;
    wire        ans      = (outst != 0) && mrsp_valid;
    wire        ans_keep = ans && (drop == 0);

    integer n;
    always @(posedge clk_sys) begin
        mc_we <= 1'b0;
        wr_v  <= 1'b0;

        // ---- memory port: issue the request FIFO head, two outstanding
        if (port_take) r_valid <= 1'b0;
        if (!r_valid && rq_cnt != 0 && (outst - (ans ? 2'd1 : 2'd0)) < 2'd2 && !t_abandon) begin
            r_valid <= 1'b1;
            r_off   <= rq[rq_rd][25:0];
            r_mask  <= rq[rq_rd][26];
        end
        outst <= outst + (port_take ? 2'd1 : 2'd0) - (ans ? 2'd1 : 2'd0);
        if (ans && drop != 0) drop <= drop - 3'd1;
        if (ans_keep && mrsp_err) mem_error <= 1'b1;
        if (ans_keep) begin
            aq[aq_wr] <= mrsp_err ? 64'd0 : mrsp_line;
            aq_wr <= aq_wr + 2'd1;
        end
        // request FIFO pops when the request is posted
        if (!r_valid && rq_cnt != 0 && (outst - (ans ? 2'd1 : 2'd0)) < 2'd2 && !t_abandon) rq_rd <= rq_rd + 3'd1;

        // ---- writer: one step per clock
        begin
            logic pop_a, pop_j;
            pop_a = 1'b0; pop_j = 1'b0;
            if (w_ready) begin
                if (w_can_m) begin
                    // mask answer: keep it, fill the cache, then the pixel (if fetched) next clock
                    w_mword <= aq[aq_rd];
                    mc_we <= 1'b1;
                    mc_wa <= {j_mkey[8:2], j_mkey[1:0]};
                    mc_wd <= {1'b1, j_mkey[17:9], aq[aq_rd]};
                    pop_a = 1'b1;
                    if (j_pf) w_step <= 1'b1;
                    else begin
                        pop_j = 1'b1;
                        wr_v <= aq[aq_rd][63 - j_mbit];
                        wr_l <= j_l; wr_a <= {ln_buf, j_x}; wr_d <= {1'b1, j_pri, j_col, w_pword[63 - 8 * j_pbyte -: 8]};
                    end
                end else begin
                    if (w_can_p) begin
                        w_pword <= aq[aq_rd];
                        pop_a = 1'b1;
                    end
                    pop_j = 1'b1;
                    w_step <= 1'b0;
                    wr_v <= w_op;
                    wr_l <= j_l; wr_a <= {ln_buf, j_x}; wr_d <= {1'b1, j_pri, j_col, w_pix};
                end
            end
            if (pop_a) aq_rd <= aq_rd + 2'd1;
            if (pop_j) jq_rd <= jq_rd + 3'd1;
            aq_cnt <= aq_cnt + (ans_keep ? 3'd1 : 3'd0) - (pop_a ? 3'd1 : 3'd0);
            jq_cnt <= jq_cnt + ((wst == W_C && jq_push_ok && line_active) ? 4'd1 : 4'd0) - (pop_j ? 4'd1 : 4'd0);
            rq_cnt <= rq_cnt + ((wst == W_C && jq_push_ok && line_active) ?
                               ((((!mc_hit && !(lk_m_v && lk_m == b_mkey)) ? 1 : 0) +
                                 ((!(lk_p_v && lk_p == b_pkey) && !(mc_hit && !mc_bit)) ? 1 : 0))) : 4'd0)
                     - ((!r_valid && rq_cnt != 0 && (outst - (ans ? 2'd1 : 2'd0)) < 2'd2 && !t_abandon) ? 4'd1 : 4'd0);
        end

        // ---- walker
        case (wst)
            W_CLEAR: begin
                mc_we <= 1'b1;
                mc_wa <= clr_idx;
                mc_wd <= '0;
                clr_idx <= clr_idx + 9'd1;
                if (clr_idx == 9'd511) wst <= W_IDLE;
            end
            W_LAYER: begin
                // start layer `lay`: enabled? per-line parameters?
                lk_m_v <= 1'b0; lk_p_v <= 1'b0;
                if (ln_ctl[16*(8*lay + 1) + 15]) begin
                    if (lay) wst <= W_DONE;
                    else lay <= 1'b1;
                end else if (lay && ln_ctl[15:0] == 16'h8000) begin
                    lay_scan <= 1'b1;
                    prm_k <= '0;
                    vram_addr <= 16'h7040 + {5'd0, ln_y[7:3], 7'd0} + {10'd0, ln_y[2:0], 3'd0};
                    wst <= W_PRM;
                end else begin
                    lay_scan <= 1'b0;
                    for (n = 0; n < 8; n++) pw[n] <= ln_ctl[16*(8*lay + n) +: 16];
                    wst <= W_SETUP;
                end
            end
            W_PRM: begin                    // VRAM word k requested; data next clock
                vram_addr <= vram_addr + 16'd1;
                wst <= W_PRMW;
            end
            W_PRMW: begin
                pw[prm_k] <= vram_data;
                prm_k <= prm_k + 3'd1;
                if (prm_k == 3'd7) wst <= W_SETUP;
                else begin vram_addr <= vram_addr + 16'd1; end
            end
            W_SETUP: begin
                if (lay_scan && pw[1][15]) begin       // this line's parameters disable the layer
                    if (lay) wst <= W_DONE; else begin lay <= 1'b1; wst <= W_LAYER; end
                end else begin
                    sm     <= (12'd512 << pw[1][9:8]) - 12'd1;
                    p_col  <= pw[1][3:0];
                    p_pri  <= pw[1][7:4];
                    p_left <= {pw[2][14:12], 9'd0};
                    p_top  <= {pw[3][14:12], 9'd0};
                    incxx  <= s13(pw[2]);
                    incxy  <= s13(pw[3]);
                    incyx  <= s13(pw[4]);
                    incyy  <= s13(pw[5]);
                    m_x    <= y3 * s13(pw[4]);
                    m_y    <= y3 * s13(pw[5]);
                    wst    <= W_MUL;
                end
            end
            W_MUL: begin
                X  <= {pw[6], 4'd0} + 20'(36 * incxx) + m_x;
                Y  <= {pw[7], 4'd0} + 20'(36 * incxy) + m_y;
                px <= '0;
                wst <= W_A;
            end
            W_A: begin                      // position -> VRAM address
                if (px == 9'd288) begin
                    if (lay) wst <= W_DONE; else begin lay <= 1'b1; wst <= W_LAYER; end
                end else begin
                    xpos <= xpos_c;
                    ypos <= ypos_c;
                    vram_addr <= vidx;
                    wst <= W_V;
                end
            end
            W_V: wst <= W_B;                 // VRAM: address registered, data registered (2 clocks)
            W_B: begin                      // tile code -> keys, cache index (mc_ra from mangle/ypos)
                b_pkey  <= {ptile[14:0], ypos[3:0], xpos[3]};
                b_mkey  <= {mtile, ypos[3:2]};
                b_mbit  <= {ypos[1:0], xpos[3], xpos[2:0]};
                b_pbyte <= xpos[2:0];
                wst <= W_C;
            end
            W_C: begin                      // cache answer -> job + reads
                if (jq_push_ok) begin
                    automatic logic need_m, need_p, known;
                    automatic logic [1:0] msrc;
                    known  = mc_hit;
                    need_m = !mc_hit && !(lk_m_v && lk_m == b_mkey);
                    need_p = !(lk_p_v && lk_p == b_pkey) && !(mc_hit && !mc_bit);
                    msrc   = mc_hit ? 2'd0 : need_m ? 2'd1 : 2'd2;
                    jq[jq_wr] <= {px, lay, p_pri, p_col, msrc, mc_bit, b_mbit, need_p, b_pbyte, b_mkey, 1'b0};
                    jq_wr <= jq_wr + 3'd1;
                    begin
                        automatic logic [2:0] w = rq_wr;
                        if (need_m) begin
                            rq[w] <= {1'b1, 5'd0, b_mkey[17:2], b_mkey[1:0], 3'b000};
                            w = w + 3'd1;
                            lk_m <= b_mkey; lk_m_v <= 1'b1;
                        end
                        if (need_p) begin
                            rq[w] <= {1'b0, 3'd0, b_pkey[19:5], b_pkey[4:1], b_pkey[0], 3'b000};
                            w = w + 3'd1;
                            lk_p <= b_pkey; lk_p_v <= 1'b1;
                        end
                        rq_wr <= w;
                        reads_cnt <= reads_cnt + {14'd0, need_m} + {14'd0, need_p};
                    end
                    X  <= X + incxx;
                    Y  <= Y + incxy;
                    px <= px + 9'd1;
                    wst <= W_A;
                end
            end
            default: ;                      // W_IDLE, W_DONE
        endcase

        // ---- line bookkeeping (C123 renderer rules)
        if (line_active) ln_cycles <= (ln_cycles == 16'hFFFF) ? ln_cycles : ln_cycles + 16'd1;
        if (line_done) begin
            line_active <= 1'b0;
            if (ln_cycles > worst_cnt) worst_cnt <= ln_cycles;
        end
        if (t_abandon) begin
            if (underruns != 16'hFFFF) underruns <= underruns + 16'd1;
            worst_cnt <= 16'hFFFF;
            // drop every answer still owed (posted or accepted)
            drop <= drop + {1'b0, outst} + (port_take ? 3'd1 : 3'd0) - (ans && drop != 0 ? 3'd1 : 3'd0)
                    + ((r_valid && !port_take) ? 3'd0 : 3'd0);
            jq_rd <= '0; jq_wr <= '0; jq_cnt <= '0;
            rq_rd <= '0; rq_wr <= '0; rq_cnt <= '0;
            aq_rd <= '0; aq_wr <= '0; aq_cnt <= '0;
            w_step <= 1'b0;
            line_active <= 1'b0;
            wst <= W_IDLE;
        end
        if (line_end && start_now && wst != W_CLEAR) begin
            line_active <= 1'b1;
            ln_cycles <= '0;
            ln_y    <= oflip ? 8'd223 - tgt_next[7:0] : tgt_next[7:0];   // buffer slot stays T (ln_buf)
            ln_buf  <= tgt_next[0];
            ln_ctl  <= ctl;
            ln_bank <= rbank;
            ln_swap <= swap46;
            lay     <= 1'b0;
            wst     <= W_LAYER;
            if (tgt_next == 9'd0) begin
                reads_frame <= reads_cnt;
                worst_frame <= worst_cnt;
                reads_cnt   <= '0;
                worst_cnt   <= '0;
            end
        end
        if (reset) begin
            wst <= W_CLEAR; clr_idx <= '0; line_active <= 1'b0; r_valid <= 1'b0; outst <= '0; drop <= '0;
            jq_rd <= '0; jq_wr <= '0; jq_cnt <= '0; rq_rd <= '0; rq_wr <= '0; rq_cnt <= '0;
            aq_rd <= '0; aq_wr <= '0; aq_cnt <= '0; w_step <= 1'b0;
            underruns <= '0; reads_frame <= '0; worst_frame <= '0; reads_cnt <= '0; worst_cnt <= '0;
            mem_error <= 1'b0;
        end
    end
endmodule
