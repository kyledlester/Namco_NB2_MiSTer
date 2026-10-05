// Namco NB-1 MiSTer core -- C355 sprite (object) line renderer (M11).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Behaviour = MAME 0.289 namco_c355spr for NB-1 [MAME-SOURCE; derivation and
// MAME pixel comparisons in docs/M11_RESEARCH.md]. The per-line formulation
// used here is scripts/research/nb1_m11_ref.py hw_render(), proven bit-equal
// to MAME's frame renderer on 259 captured frames.
//
// Frame pipeline (MAME set_buffer(2)):
//   vblank V   (line 224) CAPTURE: copy what the C355 needs from sprite RAM
//              -- clip windows, the display list, per listed sprite its
//              attributes and format entry, and its tile-table cells -- plus
//              the tile bank into snapshot buffer `cbuf`. CPU sprite-RAM
//              writes are held while this runs (`capture_busy`; Nebulas Ray
//              never writes before line 233).
//   vblank V+1 COMMIT: that buffer becomes the display buffer, the position
//              registers are latched, and lines 0..223 of the next frame are
//              drawn from it. So a frame shows the sprite RAM of the vblank
//              two vblanks before the one that ends it (MAME: copy, render,
//              compose), with the position of the vblank that starts it.
//
// Line renderer (per line, sprites in list order; later sprites overwrite):
//   walker   per sprite: position/anchor/clip set-up and a vertical reject;
//            per grid row: MAME's exact size distribution (th = rem/rows,
//            screen height sh = round(zoom*16)) and, for rows crossing the
//            line, the source row; per column the same horizontally, the
//            tile cell, bank + offset code, the clipped screen span and the
//            16.16 source index at both ends. Division by 1..16 is a multiply
//            by ceil(2^27/k); 2^20/n is a ROM (1024 x 21).
//   fetcher  one or two 8-byte OBJ reads (the halves of the 16-pixel source
//            row that the span samples) as nb1_memory client, lowest priority.
//            M23: two job slots in a ring and up to two reads outstanding
//            (nb1_memory PIPE), so the reads of consecutive halves and jobs
//            follow each other at the controller's pace; jobs are handed to
//            the writer in list order, exactly as before.
//   writer   one pixel per clk_sys; pen $FF transparent; entry =
//            pri << 12 | (palette*256 + pixel); $FFFF = empty.
// M20 native (game) flip [INFERRED, docs/M20_RESEARCH.md]: C355 position word 2 ($620004), which MAME
// stores and never reads, is the C355's screen flip. Nebulas Ray writes 3 with its operator FLIP on and 0
// otherwise; it then moves the sprites by the scroll registers but leaves every sprite's own flip bits
// alone. Without it MAME (and M11-M19) drew no sprites at all in that mode. Latched per frame with the
// position: bit 0 mirrors X, bit 1 mirrors Y (assignment inferred; only 3 has been seen). Display line Y
// shows MAME sprite line KY - Y, and a pixel MAME would draw at x lands at KX - x, with KX = 436,
// KY = 462 in MAME's sprite coordinates (visible window [149, 436] x [239, 462]). Model-proven pixel-exact
// against MAME's unflipped frames rotated 180 degrees (scripts/research/nb1_m20_flip.py).
// M21 EXPERIMENTAL continuous zoom (zoom_cont = 1, OSD; default 0 = MAME) [docs/M21_RESEARCH.md]:
// instead of MAME's per-tile shares with a per-tile restart of the source index, one accumulator per axis
// runs across the whole sprite. For an axis of n tiles drawn at size S: step = 65536 when S == 16n, else
// (n * floor(2^28 / S)) >> 8 (= MAME's floor(2^20 / S) for n = 1); logical pixel j (0 .. S-1 from the
// source start; flips reverse it) shows source column min((j * step) >> 16, 16n - 1). Tile t covers the
// logical span [ceil(t * zx), ceil((t+1) * zx)) with zx = (S << 12) / n (the last tile ends at S), so each
// tile is still one fetch job. Unverified against hardware: an experiment, not the reference behaviour.
// Four line buffers (a ring): the renderer may run up to 3 lines ahead of
// the display, and draws lines 0-3 during vblank. A line not finished when
// its display starts is abandoned (counted in `drops`, shown as drawn).
module nb1_c355_render #(parameter int CELL_AW = 12) (
    input  wire         clk_sys,
    input  wire         reset,

    input  wire         line_end,
    input  wire         vblank_begin,
    input  wire         zoom_cont,          // M21 experiment: 1 = continuous zoom (latched per frame)
    input  wire [8:0]   vcount,

    // sprite RAM snapshot port (nb1_c355_ram)
    output reg  [14:0]  snap_addr = '0,
    input  wire [15:0]  snap_data,
    input  wire [63:0]  pos_in,
    input  wire [127:0] bank_in,
    output wire         capture_busy,

    // nb1_memory client (reads of region OBJ)
    output reg          mreq_valid = 1'b0,
    input  wire         mreq_ready,
    output wire [3:0]   mreq_region,
    output reg  [24:0]  mreq_offset = '0,
    output wire         mreq_we,
    output wire [31:0]  mreq_wdata,
    output wire [3:0]   mreq_be,
    input  wire         mrsp_valid,
    input  wire [63:0]  mrsp_line,
    input  wire         mrsp_err,

    // display port (nb1_video_out)
    input  wire         disp_rd,
    input  wire [1:0]   disp_slot,          // line & 3
    input  wire [8:0]   disp_x,
    output wire [15:0]  disp_entry,         // valid the clk after disp_rd; $FFFF = no sprite

    // diagnostics
    output reg  [8:0]   spr_count = '0,      // sprites in the displayed list
    output reg  [12:0]  cell_count = '0,     // cells captured for it
    output reg          overflow = 1'b0,     // a capture ran out of cell space
    output reg  [15:0]  drops = '0,          // lines abandoned (saturating)
    output reg  [15:0]  drop_frames = '0,    // frames with at least one abandoned line (saturating)
    output reg  [7:0]   drop_max = '0,       // most lines abandoned in one frame (saturating)
    output reg  [15:0]  fetch_frame = '0,    // OBJ reads in the last frame
    output reg          mem_error = 1'b0
);
    import nb1_mem_pkg::*;
    localparam int CELLS = 1 << CELL_AW;

    assign mreq_region = REG_OBJ;
    assign mreq_we     = 1'b0;
    assign mreq_wdata  = '0;
    assign mreq_be     = 4'hF;

    // ------------------------------------------------------------ helpers
    function automatic signed [11:0] sext9(input [15:0] v);
        sext9 = {{3{v[8]}}, v[8:0]};
    endfunction
    function automatic [26:0] mk(input [4:0] k);   // ceil(2^27 / k), k 2..16
        case (k)
            5'd2: mk = 27'd67108864;  5'd3: mk = 27'd44739243;  5'd4: mk = 27'd33554432;
            5'd5: mk = 27'd26843546;  5'd6: mk = 27'd22369622;  5'd7: mk = 27'd19173962;
            5'd8: mk = 27'd16777216;  5'd9: mk = 27'd14913081;  5'd10: mk = 27'd13421773;
            5'd11: mk = 27'd12201612; 5'd12: mk = 27'd11184811; 5'd13: mk = 27'd10324441;
            5'd14: mk = 27'd9586981;  5'd15: mk = 27'd8947849;  5'd16: mk = 27'd8388608;
            default: mk = 27'd0;
        endcase
    endfunction

    // 2^28 / n, n = 1..1023 (index 0 unused). M21: widened from 2^20 / n (same 3 M10K); the MAME path uses
    // bits [27:8] = floor(2^20 / n) exactly.
    (* ramstyle = "M10K" *) reg [27:0] recip_rom [0:1023];
    initial begin
        recip_rom[0] = 28'd0;
        for (int i = 1; i < 1024; i++) recip_rom[i] = 28'((1 << 28) / i);
    end
    reg  [9:0]  rc_addr = '0;
    reg  [27:0] rc_q28 = '0;
    always @(posedge clk_sys) rc_q28 <= recip_rom[rc_addr];
    wire [20:0] rc_q = {1'b0, rc_q28[27:8]};

    // ------------------------------------------------------ snapshot stores
    // raw descriptor (110 bits): X[10:0] Y[10:0] W{flip,9:0} H{flip,9:0}
    // pal{clip,pri,col} off fmt dx dy cellbase
    localparam int DW = 11 + 11 + 11 + 11 + 12 + 16 + 8 + 9 + 9 + 12;
    (* ramstyle = "M10K, no_rw_check" *) reg [DW-1:0] desc_mem [0:511];
    reg [8:0]    desc_waddr = '0, desc_raddr = '0;
    reg [DW-1:0] desc_wdata = '0, desc_q = '0;
    reg          desc_we = 1'b0;
    always @(posedge clk_sys) begin
        if (desc_we) desc_mem[desc_waddr] <= desc_wdata;
        desc_q <= desc_mem[desc_raddr];
    end

    (* ramstyle = "M10K, no_rw_check" *) reg [15:0] cell_mem [0:2*CELLS-1];
    reg [CELL_AW:0] cell_waddr = '0, cell_raddr = '0;
    reg [15:0]      cell_wdata = '0, cell_q = '0;
    reg             cell_we = 1'b0;
    always @(posedge clk_sys) begin
        if (cell_we) cell_mem[cell_waddr] <= cell_wdata;
        cell_q <= cell_mem[cell_raddr];
    end

    (* ramstyle = "MLAB, no_rw_check" *) reg [63:0] clip_mem [0:31];   // {x0, x1, y0, y1}
    reg [4:0]  clip_waddr = '0, clip_raddr = '0;
    reg [63:0] clip_wdata = '0, clip_q = '0;
    reg        clip_we = 1'b0;
    always @(posedge clk_sys) begin
        if (clip_we) clip_mem[clip_waddr] <= clip_wdata;
        clip_q <= clip_mem[clip_raddr];
    end

    (* ramstyle = "logic" *) reg [127:0] bank_s [0:1];   // M16: tiny; kept out of M10K
    reg [8:0]   count_s [0:1];
    reg [12:0]  cells_s [0:1];

    // ------------------------------------------------------------ capture
    localparam [3:0] C_IDLE = 4'd0, C_CLIP = 4'd1, C_LIST = 4'd2, C_ATTR = 4'd3, C_FMT = 4'd4,
                     C_CELL = 4'd5, C_DESC = 4'd6, C_PREP = 4'd7;
    reg [3:0]  cst = C_IDLE;
    reg        cbuf = 1'b0;           // buffer being captured
    reg        cph = 1'b0;            // 0 = address issued, 1 = data (one read per 2 clk)
    reg        cwait = 1'b0;
    reg [5:0]  cclip = '0;            // clip word 0..63
    reg [63:0] cclip_acc = '0;
    reg [7:0]  cidx = '0;             // list index
    reg        cend = 1'b0;
    reg [7:0]  cwhich = '0;
    reg [2:0]  cj = '0;
    reg [15:0] ca [0:6];
    reg [15:0] cf [0:3];
    reg [8:0]  cn = '0;               // cells of this sprite
    reg [8:0]  ck = '0;
    reg [CELL_AW:0] cptr = '0;
    reg        cskip = 1'b0;
    reg [15:0] ctbase = '0;           // $4000 + tile index (C_PREP)
    reg        c_ovf = 1'b0;
    assign capture_busy = (cst != C_IDLE);

    wire [4:0] c_ncol = (cf[1][7:4] == 4'd0) ? 5'd16 : {1'b0, cf[1][7:4]};
    wire [4:0] c_nrow = (cf[1][3:0] == 4'd0) ? 5'd16 : {1'b0, cf[1][3:0]};
    wire [15:0] c_taddr = ctbase + {7'd0, ck};
    wire        c_off   = (ca[4][9:0] == 10'd0) || (ca[5][9:0] == 10'd0);
    // M25: far off-screen sprites need no cells. J-League Soccer V-Shoot draws its pitch as a 5 x 3 grid of
    // 256 x 256 sprites (16 x 16 cells each) of which only ~4 are on screen: 4156 cells in a frame against
    // CELLS = 4096, which would drop the last (topmost) sprites of the list. On an axis whose format offset
    // (dx/dy, zoom-scaled by the walker) is 0 the screen position is exactly sext11(pos - xs/ys) (MAME
    // get_single_sprite); a sprite more than 16 pixels outside the 288 x 224 screen on such an axis draws
    // nothing (the margin also covers the position registers moving between this capture and the draw latch).
    // It is stored like an out-of-space sprite: width 0, no cells.
    wire signed [11:0] c_xs = sext9(pos_in[31:16]) + 12'sd38;
    wire signed [11:0] c_ys = sext9(pos_in[15:0])  + 12'sd25;
    wire [15:0]        c_hd = ca[2] - {{4{c_xs[11]}}, c_xs};
    wire [15:0]        c_vd = ca[3] - {{4{c_ys[11]}}, c_ys};
    wire signed [11:0] c_hp = {c_hd[10], c_hd[10:0]};                 // sext11
    wire signed [11:0] c_vp = {c_vd[10], c_vd[10:0]};
    wire signed [12:0] c_hr = c_hp + $signed({3'd0, ca[4][9:0]});     // right / bottom edge (exclusive)
    wire signed [12:0] c_vb = c_vp + $signed({3'd0, ca[5][9:0]});
    wire        c_far   = ((cf[2][7:0] == 8'd0) && (c_hp >= 12'sd304 || c_hr <= -13'sd16)) ||
                          ((cf[3][7:0] == 8'd0) && (c_vp >= 12'sd240 || c_vb <= -13'sd16));
    reg         cfar    = 1'b0;

    // ------------------------------------------------------------ display state
    reg        dbuf = 1'b0;           // buffer being displayed
    reg [8:0]  dcount = '0;
    reg [127:0] dbank = '0;
    reg signed [11:0] xs = '0, ys = '0;
    reg               flx = 1'b0, fly = 1'b0;          // M20: C355 screen flip (position word 2), per frame

    // ------------------------------------------------------------ line ring
    reg        dclr = 1'b0;
    reg [10:0] dclr_addr = '0;
    always @(posedge clk_sys) begin
        dclr      <= disp_rd;
        dclr_addr <= {disp_slot, disp_x};
    end
    (* ramstyle = "M10K, no_rw_check" *) reg [15:0] ring [0:2047];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 2048; i++) ring[i] = 16'hFFFF;
`endif
// synthesis translate_on
    reg  [15:0] ring_q = 16'hFFFF;
    wire        wr_on;
    wire [10:0] wr_addr;
    wire [15:0] wr_data;
    wire        ring_we    = dclr || wr_on;
    wire [10:0] ring_waddr = dclr ? dclr_addr : wr_addr;
    wire [15:0] ring_wdata = dclr ? 16'hFFFF : wr_data;
    always @(posedge clk_sys) begin
        if (ring_we) ring[ring_waddr] <= ring_wdata;
        if (disp_rd) ring_q <= ring[{disp_slot, disp_x}];
    end
    assign disp_entry = ring_q;

    // ------------------------------------------------------------ line control
    reg        line_active = 1'b0;
    reg [7:0]  rline = '0;            // line being / next to be rendered
    reg [8:0]  ldisp = '0;            // lines of this frame already displayed
    reg        frame_on = 1'b0;       // a committed frame is being drawn
    reg [7:0]  drops_cur = '0;        // lines abandoned in the frame being drawn
    wire [8:0] disp_next = (vcount == 9'd263) ? 9'd0 : vcount + 9'd1;
    wire abandon_now = line_end && frame_on && disp_next < 9'd224 && {1'b0, rline} == disp_next;   // M23

    // ------------------------------------------------------------ walker
    localparam [4:0] W_IDLE = 5'd0, W_SRD = 5'd1, W_SWT = 5'd2, W_SA = 5'd3, W_SB = 5'd4, W_SC = 5'd5,
                     W_SD = 5'd6, W_RQ = 5'd7, W_RE = 5'd8, W_RDY = 5'd9, W_RDW = 5'd10, W_RY = 5'd11,
                     W_CQ = 5'd13, W_CE = 5'd14, W_CR = 5'd15, W_CDW = 5'd16, W_CX = 5'd17,
                     W_CX2 = 5'd18, W_CP = 5'd19, W_DONE = 5'd20, W_NEXTS = 5'd21,
                     W_SE = 5'd22, W_SF = 5'd23, W_RF = 5'd12,
                     // M21 continuous path
                     W_KA = 5'd24, W_KB = 5'd25, W_KC = 5'd26, W_KD = 5'd27, W_KE = 5'd28,
                     W_KF = 5'd29, W_KG = 5'd30, W_KH = 5'd31;
    localparam [5:0] W_KI = 6'd32, W_KJ = 6'd33, W_KK = 6'd34, W_KL = 6'd35, W_KM = 6'd36, W_KN = 6'd37,
                     W_KQ = 6'd38, W_KR = 6'd39;
    reg [5:0]  wst = {1'b0, W_IDLE};
    reg [8:0]  n = '0;
    // sprite set-up (positions are 16-bit signed: MAME uses int)
    reg signed [15:0] hp = '0, vp = '0, hpos = '0;
    reg [9:0]  wsz = '0, hsz = '0;
    reg        fx = 1'b0, fy = 1'b0, s_en = 1'b0;
    reg [4:0]  ncol = '0, nrow = '0;
    reg [3:0]  scol = '0, spri = '0;
    reg [15:0] soff = '0;
    reg [8:0]  sdx = '0, sdy = '0;
    reg [CELL_AW-1:0] cbase = '0;
    reg [21:0] zx = '0, zy = '0;
    reg [48:0] mzx = '0, mzy = '0;
    reg [29:0] azx = '0, azy = '0;
    reg signed [17:0] kx0 = '0, kx1 = '0, ky0 = '0, ky1 = '0;   // clip window, screen space
    reg signed [15:0] cx0 = '0, cx1 = '0;                       // clamped to 0..287
    // set-up pipeline (timing, M11 closeout): W_SD rounds the anchor and clamps
    // the clip window, W_SE forms the positions, W_SF decides
    reg signed [15:0] axr = '0, ayr = '0, vpn_r = '0, tp_r = '0;
    reg signed [17:0] cy0 = '0, cy1 = '0;
    reg               cxe = 1'b0;                               // clip x range empty
    // row loop
    reg [4:0]  r = '0, remr = '0, rcur = '0;
    reg [9:0]  remh = '0, sh = '0;
    reg signed [15:0] yy = '0, ty = '0;
    reg [48:0] mr = '0;
    reg [31:0] my = '0;
    reg [9:0]  krow = '0;             // M25 (timing): the source-row index, registered one state before the multiply
    reg [3:0]  srow = '0;
    reg        rhit = 1'b0;                                     // W_RE -> W_RF: this row crosses the line
    // column loop
    reg [4:0]  c = '0, remc = '0;
    reg [9:0]  remw = '0, sw = '0;
    reg signed [15:0] xx = '0, tx = '0;
    reg        qx_nz = 1'b0;
    reg [48:0] mc = '0;
    reg [31:0] mxa = '0, mxb = '0;
    reg signed [15:0] jx0 = '0, jx1 = '0;
    // M21 continuous zoom
    reg               zc = 1'b0;                 // latched zoom_cont for the frame being drawn
    reg        [24:0] stepx = '0, stepy = '0;    // source 16.16 per destination pixel
    reg        [9:0]  kk = '0;                   // line - sprite top
    reg        [35:0] kmul = '0;
    reg        [21:0] aa = '0;                   // tile boundary accumulator (20.12, destination pixels)
    reg        [9:0]  a_lo = '0, a_hi = '0;      // logical span of the current tile
    reg signed [15:0] le = '0;                   // left edge of the sprite on screen
    reg signed [15:0] dx0 = '0, dx1 = '0;        // destination span of the current tile
    reg        [35:0] kma = '0, kmb = '0;
    reg        [27:0] rqy = '0, rqx = '0;         // M21 timing registers
    reg        [32:0] py = '0, px = '0;
    reg        [9:0]  j0r = '0, j1r = '0;
    reg        [9:0]  ix0 = '0, ix1 = '0;         // M21 timing: MAME-path span-end indices
    reg        [36:0] ta_r = '0, tb_r = '0;
    reg [15:0] jcode = '0;
    reg [20:0] jdx = '0;
    // M20: the MAME sprite line evaluated for display line rline (registered: first used 4+ clocks after
    // rline changes, in W_SF)
    reg  signed [15:0] ln = '0;
    always @(posedge clk_sys) ln <= fly ? 16'sd462 - $signed({8'd0, rline}) : $signed({8'd0, rline});

    function automatic [21:0] divq(input [48:0] prod, input [21:0] nn, input [4:0] k);
        divq = (k == 5'd1) ? nn : prod[48:27];
    endfunction

    // desc field views
    wire [10:0] d_X = desc_q[109:99];
    wire [10:0] d_Y = desc_q[98:88];
    wire [10:0] d_W = desc_q[87:77];
    wire [10:0] d_H = desc_q[76:66];
    wire [11:0] d_pal = desc_q[65:54];
    wire [15:0] d_off = desc_q[53:38];
    wire [7:0]  d_fmt = desc_q[37:30];
    wire [8:0]  d_dx = desc_q[29:21];
    wire [8:0]  d_dy = desc_q[20:12];
    wire [11:0] d_cb = desc_q[11:0];

    // ------------------------------------------------------------ job FIFO
    // {code16, srow4, x0 10, x1 10, xi 21, step 21, fx, col4, pri4, halves2}
    localparam int JW = 16 + 4 + 10 + 10 + 21 + 21 + 1 + 4 + 4 + 2;
    (* ramstyle = "logic" *) reg [JW-1:0] jf [0:3];   // M16: tiny; kept out of M10K
    reg [1:0] jf_rd = '0, jf_wr = '0;
    reg [2:0] jf_cnt = '0;
    wire      jf_full = (jf_cnt == 3'd4), jf_empty = (jf_cnt == 3'd0);
    wire [JW-1:0] jf_head = jf[jf_rd];
    wire      jf_push = (wst == W_CP) && !jf_full;
    wire      jf_pop;

    // ------------------------------------------------------------ fetcher
    // M23: slot s holds job fsj[s] and its row fsd[s]; fs_need = halves still to be issued
    // {lo, hi}; fs_wait = halves issued and not yet answered. flp = next slot to load,
    // fip = slot being issued, fhp = next slot for the writer (all advance in ring order).
    reg [JW-1:0] fsj [0:1];
    reg [127:0]  fsd [0:1];
    reg [1:0]    fs_full = '0;
    reg [1:0]    fs_need [0:1];
    reg [1:0]    fs_wait [0:1];
    reg          flp = 1'b0, fip = 1'b0, fhp = 1'b0;
    reg [1:0]    ftq [0:1];                      // issued reads in order: {slot, half (1 = hi)}
    reg          ftq_rd = 1'b0, ftq_wr = 1'b0;
    reg [1:0]    f_out = '0;                    // accepted by nb1_memory, not yet answered
    reg [1:0]    f_disc = '0;                     // oldest responses to drop (abandoned line)
    wire         mem_pending = mreq_valid || (f_out != 2'd0);
    wire         rsp_mine = (f_out != 2'd0) && mrsp_valid;
    wire         slot_rdy = fs_full[fhp] && (fs_need[fhp] == 2'd0) && (fs_wait[fhp] == 2'd0);
    wire         fetch_idle = (fs_full == 2'd0);
    reg  [15:0]  fetch_cnt = '0;
    initial begin fs_need[0] = '0; fs_need[1] = '0; fs_wait[0] = '0; fs_wait[1] = '0; end

    // ------------------------------------------------------------ writer
    reg          wr_busy = 1'b0;
    reg [9:0]    wr_x = '0, wr_x1 = '0;
    reg [8:0]    wr_a = '0;                     // M20: line-buffer address (KX - x when X is mirrored)
    reg          wr_mx = 1'b0;
    reg [20:0]   wr_xi = '0, wr_step = '0;
    reg          wr_fx = 1'b0;
    reg [3:0]    wr_col = '0, wr_pri = '0;
    reg [127:0]  wr_row = '0;
    reg [1:0]    wr_slot = '0;
    reg          wr_zc = 1'b0;                  // M21: this job is continuous-zoom (clamped index)
    wire [3:0]   wr_sc = (wr_zc && wr_xi[20]) ? 4'd15 : wr_xi[19:16];
    wire [7:0]   wr_pix = wr_row[127 - 8 * wr_sc -: 8];
    assign wr_on   = wr_busy && !dclr && (wr_pix != 8'hFF);
    assign wr_addr = {wr_slot, wr_a};
    assign wr_data = {wr_pri, wr_col, wr_pix};
    wire   wr_load = !wr_busy && slot_rdy;
    assign jf_pop  = !fs_full[flp] && !jf_empty && line_active;

    wire line_done = line_active && (wst == W_DONE) && jf_empty && fetch_idle && !wr_busy && !mem_pending;

    always @(posedge clk_sys) begin
        desc_we <= 1'b0;
        cell_we <= 1'b0;
        clip_we <= 1'b0;

        // ========================================================== capture
        case (cst)
            C_IDLE: ;
            C_CLIP: begin
                if (!cph) begin snap_addr <= 15'h1200 + {9'd0, cclip}; cph <= 1'b1; cwait <= 1'b1; end
                else if (cwait) cwait <= 1'b0;
                else begin
                    cph <= 1'b0;
                    cclip_acc <= {cclip_acc[47:0], snap_data};
                    if (cclip[1:0] == 2'd3) begin
                        clip_we    <= 1'b1;
                        clip_waddr <= {cbuf, cclip[5:2]};
                        clip_wdata <= {cclip_acc[47:0], snap_data};
                    end
                    cclip <= cclip + 6'd1;
                    if (cclip == 6'd63) begin cst <= C_LIST; cidx <= 8'd0; cptr <= '0; c_ovf <= 1'b0; end
                end
            end
            C_LIST: begin
                if (!cph) begin snap_addr <= 15'h1000 + {7'd0, cidx}; cph <= 1'b1; cwait <= 1'b1; end
                else if (cwait) cwait <= 1'b0;
                else begin
                    cph <= 1'b0; cwhich <= snap_data[7:0]; cend <= snap_data[8] || (cidx == 8'd255);
                    cj <= 3'd0; cst <= C_ATTR;
                end
            end
            C_ATTR: begin
                if (!cph) begin snap_addr <= {4'd0, cwhich, cj}; cph <= 1'b1; cwait <= 1'b1; end
                else if (cwait) cwait <= 1'b0;
                else begin
                    cph <= 1'b0; ca[cj] <= snap_data;
                    if (cj == 3'd6) begin cj <= 3'd0; cst <= C_FMT; end
                    else cj <= cj + 3'd1;
                end
            end
            C_FMT: begin
                if (!cph) begin snap_addr <= 15'h2000 + {2'd0, ca[0][10:0], cj[1:0]}; cph <= 1'b1; cwait <= 1'b1; end
                else if (cwait) cwait <= 1'b0;
                else begin
                    cph <= 1'b0; cf[cj[1:0]] <= snap_data;
                    if (cj == 3'd3) begin cst <= C_PREP; ck <= 9'd0; end
                    else cj <= cj + 3'd1;
                end
            end
            C_PREP: begin
                // cell count and space check for this sprite, registered (timing)
                // a disabled sprite (width or height 0) reads no cells, like MAME
                cn     <= (c_off || c_far) ? 9'd0 : {4'd0, c_ncol} * {4'd0, c_nrow};
                cskip  <= !c_off && !c_far && ({1'b0, cptr} + {5'd0, c_ncol} * {5'd0, c_nrow} > CELLS);
                cfar   <= c_far;
                ctbase <= 16'h4000 + cf[0];
                cst    <= C_CELL;
            end
            C_CELL: begin
                if (cskip || ck == cn) cst <= C_DESC;
                else if (!cph) begin snap_addr <= c_taddr[14:0]; cph <= 1'b1; cwait <= 1'b1; end
                else if (cwait) cwait <= 1'b0;
                else begin
                    cph        <= 1'b0;
                    cell_we    <= 1'b1;
                    cell_waddr <= {cbuf, CELL_AW'(cptr + {4'd0, ck})};
                    cell_wdata <= c_taddr[15] ? 16'h0000 : snap_data;
                    ck         <= ck + 9'd1;
                end
            end
            C_DESC: begin
                desc_we    <= 1'b1;
                desc_waddr <= {cbuf, cidx};
                // an out-of-space sprite is stored with width 0 (drawn as disabled)
                desc_wdata <= {ca[2][10:0], ca[3][10:0], ca[4][15], ((cskip || cfar) ? 10'd0 : ca[4][9:0]), ca[5][15], ca[5][9:0],
                               ca[6][11:0], ca[1], cf[1][7:0], cf[2][8:0], cf[3][8:0], 12'(cptr)};
                if (cskip) c_ovf <= 1'b1;
                else cptr <= cptr + CELL_AW'(cn);
                if (cend) begin
                    count_s[cbuf] <= {1'b0, cidx} + 9'd1;
                    cells_s[cbuf] <= 13'(cskip ? cptr : cptr + CELL_AW'(cn));
                    if (cskip || c_ovf) overflow <= 1'b1;
                    cst <= C_IDLE;
                end else begin
                    cidx <= cidx + 8'd1;
                    cst  <= C_LIST;
                end
            end
            default: cst <= C_IDLE;
        endcase

        // ========================================================== memory port
        // (M23: the outstanding count, the tag queue and the discards are kept by the fetcher below)

        // ========================================================== job FIFO
        if (jf_push) begin
            jf[jf_wr] <= {jcode, srow, jx0[9:0], jx1[9:0], mxa[20:0], jdx, fx, scol, spri,
                          (mxa[19:16] < 4'd8 || mxb[19:16] < 4'd8), (mxa[19:16] >= 4'd8 || mxb[19:16] >= 4'd8)};
            jf_wr <= jf_wr + 2'd1;
        end
        if (jf_pop) jf_rd <= jf_rd + 2'd1;
        jf_cnt <= jf_cnt + (jf_push ? 3'd1 : 3'd0) - (jf_pop ? 3'd1 : 3'd0);

        // ========================================================== walker
        case (wst)
            W_IDLE, W_DONE: ;
            W_SRD: begin desc_raddr <= {dbuf, n[7:0]}; wst <= W_SWT; end
            W_SWT: wst <= W_SA;
            W_SA: begin : sa
                reg [10:0] dxw, dyw;
                dxw = d_X - xs[10:0];                  // (X - xscroll) & $7FF, sign-extended
                dyw = d_Y - ys[10:0];
                ncol <= (d_fmt[7:4] == 4'd0) ? 5'd16 : {1'b0, d_fmt[7:4]};
                nrow <= (d_fmt[3:0] == 4'd0) ? 5'd16 : {1'b0, d_fmt[3:0]};
                wsz  <= d_W[9:0]; hsz <= d_H[9:0]; fx <= d_W[10]; fy <= d_H[10];
                s_en <= (d_W[9:0] != 10'd0) && (d_H[9:0] != 10'd0);
                scol <= d_pal[3:0]; spri <= d_pal[7:4];
                soff <= d_off; sdx <= d_dx; sdy <= d_dy; cbase <= d_cb[CELL_AW-1:0];
                hp   <= {{5{dxw[10]}}, dxw};
                vp   <= {{5{dyw[10]}}, dyw};
                mzx  <= {d_W[9:0], 12'd0} * mk((d_fmt[7:4] == 4'd0) ? 5'd16 : {1'b0, d_fmt[7:4]});
                mzy  <= {d_H[9:0], 12'd0} * mk((d_fmt[3:0] == 4'd0) ? 5'd16 : {1'b0, d_fmt[3:0]});
                clip_raddr <= {dbuf, d_pal[11:8]};
                wst  <= W_SB;
            end
            W_SB: begin
                zx  <= divq(mzx, {wsz, 12'd0}, ncol);   // zoom = (size << 16) / (count * 16)
                zy  <= divq(mzy, {hsz, 12'd0}, nrow);
                wst <= W_SC;
            end
            W_SC: begin
                azx <= sdx[7:0] * zx;
                azy <= sdy[7:0] * zy;
                kx0 <= $signed({2'b00, clip_q[63:48]}) - xs;
                kx1 <= $signed({2'b00, clip_q[47:32]}) - xs;
                ky0 <= $signed({2'b00, clip_q[31:16]}) - ys;
                ky1 <= $signed({2'b00, clip_q[15:0]})  - ys;
                wst <= W_SD;
            end
            W_SD: begin : sd
                reg signed [15:0] ax, ay;
                reg signed [17:0] a0, a1;
                ax  = $signed({1'b0, 15'((azx + 30'h8000) >> 16)});
                ay  = $signed({1'b0, 15'((azy + 30'h8000) >> 16)});
                axr <= sdx[8] ? -ax : ax;
                ayr <= sdy[8] ? -ay : ay;
                // visible window in MAME sprite coordinates: [0, 287] x [0, 223], flipped [149, 436] x [239, 462]
                if (flx) begin a0 = (kx0 < 149) ? 18'sd149 : kx0;  a1 = (kx1 > 436) ? 18'sd436 : kx1; end
                else     begin a0 = (kx0 < 0)   ? 18'sd0   : kx0;  a1 = (kx1 > 287) ? 18'sd287 : kx1; end
                cx0 <= a0[15:0]; cx1 <= a1[15:0];
                cxe <= (a1 < a0);
                if (fly) begin cy0 <= (ky0 < 239) ? 18'sd239 : ky0;  cy1 <= (ky1 > 462) ? 18'sd462 : ky1; end
                else     begin cy0 <= (ky0 < 0)   ? 18'sd0   : ky0;  cy1 <= (ky1 > 223) ? 18'sd223 : ky1; end
                wst <= W_SE;
            end
            W_SE: begin : se
                reg signed [15:0] vpn;
                vpn   = fy ? vp + ayr : vp - ayr;
                hpos  <= fx ? hp + axr : hp - axr;
                vpn_r <= vpn;
                tp_r  <= fy ? vpn - $signed({6'd0, hsz}) : vpn;
                wst   <= W_SF;
            end
            W_SF: begin
                // reject: disabled, empty clip, line outside the clip or the sprite's
                // conservative extent [top, top + H] (a tile may draw one row past its share)
                if (!s_en || cxe || $signed({2'b00, ln}) < cy0 || $signed({2'b00, ln}) > cy1 ||
                    ln < tp_r || ln > tp_r + $signed({6'd0, hsz})) begin
                    wst <= W_NEXTS;
                end else if (zc) begin
                    // M21 continuous: the source row comes from one product per line
                    kk      <= 10'(ln - tp_r);
                    rc_addr <= hsz;
                    wst     <= W_KA;
                end else begin
                    r <= 5'd0; remr <= nrow; remh <= hsz; yy <= vpn_r;
                    wst <= W_RQ;
                end
            end
            // ------------------------------------------------ rows (MAME order)
            W_RQ: begin mr <= {remh, 12'd0} * mk(remr); wst <= W_RE; end
            W_RE: begin : re
                reg [21:0] q;
                reg [9:0]  th, shn;
                reg signed [15:0] t;
                q   = divq(mr, {remh, 12'd0}, remr);
                th  = q[21:12];
                shn = 10'((q + 22'd2048) >> 12);
                t   = fy ? yy - $signed({6'd0, th}) : yy;
                yy   <= fy ? yy - $signed({6'd0, th}) : yy + $signed({6'd0, th});
                remh <= remh - th;
                remr <= remr - 5'd1;
                ty   <= t; sh <= shn; rcur <= r;
                r    <= r + 5'd1;
                rhit <= (q != 22'd0) && (shn != 10'd0);
                wst  <= W_RF;
            end
            W_RF: begin
                if (rhit && ln >= ty && ln < ty + $signed({6'd0, sh})) begin
                    rc_addr <= sh;
                    wst     <= W_RDY;
                end else if (rcur + 5'd1 == nrow) wst <= W_NEXTS;
                else wst <= W_RQ;
            end
            W_RDY: begin : rdy
                // source row = (fy ? sh-1-k : k) * (2^20 / sh) >> 16, k = line - ty
                // M25 (timing): k and the flip are formed here, while the reciprocal is read; the multiply
                // follows alone in W_RDW (ln, ty, sh and fy do not change between the two states)
                reg [9:0] k;
                k    = 10'(ln - ty);
                krow <= fy ? sh - 10'd1 - k : k;
                wst  <= W_RDW;
            end
            W_RDW: begin
                my  <= {1'b0, krow} * rc_q;
                wst <= W_RY;
            end
            W_RY: begin
                srow <= my[19:16];
                c <= 5'd0; remc <= ncol; remw <= wsz; xx <= hpos;
                wst <= W_CQ;
            end
            // ------------------------------------------------ columns
            W_CQ: begin
                mc <= {remw, 12'd0} * mk(remc);
                cell_raddr <= {dbuf, cbase + CELL_AW'(rcur * ncol) + CELL_AW'(c)};
                wst <= W_CE;
            end
            W_CE: begin : ce
                reg [21:0] q;
                reg [9:0]  tw, swn;
                reg signed [15:0] t;
                q   = divq(mc, {remw, 12'd0}, remc);
                tw  = q[21:12];
                swn = 10'((q + 22'd2048) >> 12);
                t   = fx ? xx - $signed({6'd0, tw}) : xx;
                xx   <= fx ? xx - $signed({6'd0, tw}) : xx + $signed({6'd0, tw});
                remw <= remw - tw;
                remc <= remc - 5'd1;
                tx   <= t; sw <= swn; qx_nz <= (q != 22'd0);
                c    <= c + 5'd1;
                wst  <= W_CR;
            end
            W_CR: begin : cr
                // cell_q valid now
                reg signed [15:0] tend;
                reg [15:0] bk;
                tend = tx + $signed({6'd0, sw}) - 16'sd1;
                bk   = dbank[{cell_q[13:11], 4'd0} +: 16];     // bank word (tile >> 11) & 7
                if (!cell_q[15] && qx_nz && sw != 10'd0 && tx <= cx1 && tend >= cx0) begin
                    jx0   <= (tx < cx0) ? cx0 : tx;
                    jx1   <= (tend > cx1) ? cx1 : tend;
                    jcode <= ({5'd0, cell_q[10:0]} | {bk[4:0], 11'd0}) + soff;
                    rc_addr <= sw;
                    wst <= W_CDW;
                end else if (c == ncol) wst <= (rcur + 5'd1 == nrow) ? W_NEXTS : W_RQ;
                else wst <= W_CQ;
            end
            W_CDW: begin : cdw
                // M21 timing: the span-end indices are registered here (rc_q arrives in W_CX anyway)
                reg [9:0] i0, i1;
                i0 = 10'(jx0 - tx);
                i1 = 10'(jx1 - tx);
                ix0 <= fx ? sw - 10'd1 - i0 : i0;
                ix1 <= fx ? sw - 10'd1 - i1 : i1;
                wst <= W_CX;
            end
            W_CX: begin
                // 16.16 source index at both ends of the clipped span (MAME x_index)
                jdx <= rc_q;
                mxa <= {1'b0, ix0} * rc_q;
                mxb <= {1'b0, ix1} * rc_q;
                wst <= W_CX2;
            end
            W_CX2: wst <= W_CP;
            W_CP: if (!jf_full) begin
                if (zc) wst <= (c == ncol) ? W_NEXTS : W_KF;
                else if (c == ncol) wst <= (rcur + 5'd1 == nrow) ? W_NEXTS : W_RQ;
                else wst <= W_CQ;
            end
            // ------------------------------------------------ M21 continuous zoom
            W_KA: wst <= (kk >= hsz) ? W_NEXTS : W_KB;           // line past the sprite (W_SF is conservative)
            W_KB: begin rc_addr <= wsz; wst <= W_KC; end
            // M21 timing: ROM output, the n * 2^28/S product and the shift each get their own clock
            W_KC: begin rqy <= rc_q28; wst <= W_KL; end         // 2^28 / hsz
            W_KL: begin rqx <= rc_q28; py <= {28'd0, nrow} * {5'd0, rqy}; wst <= W_KM; end   // 2^28 / wsz
            W_KM: begin
                stepy <= (hsz == {1'b0, nrow, 4'd0}) ? 25'h10000 : py[32:8];
                px    <= {28'd0, ncol} * {5'd0, rqx};
                wst   <= W_KN;
            end
            W_KN: begin
                stepx <= (wsz == {1'b0, ncol, 4'd0}) ? 25'h10000 : px[32:8];
                kmul  <= {26'd0, (fy ? hsz - 10'd1 - kk : kk)} * {11'd0, stepy};
                wst   <= W_KE;
            end
            W_KE: begin : ke
                reg [19:0] rr;
                rr   = (kmul[35:16] > {nrow, 4'd0} - 20'd1) ? {nrow, 4'd0} - 20'd1 : kmul[35:16];
                rcur <= 5'(rr[8:4]);
                srow <= rr[3:0];
                le   <= fx ? hpos - $signed({6'd0, wsz}) : hpos;
                c    <= 5'd0; aa <= 22'd0; a_lo <= 10'd0;
                wst  <= W_KF;
            end
            W_KF: begin : kf
                reg [21:0] an;
                reg [9:0]  hi;
                an = aa + zx;
                hi = (c + 5'd1 == ncol) ? wsz : 10'((an + 22'd4095) >> 12);
                a_hi <= hi;
                aa   <= an;
                dx0  <= fx ? le + $signed({6'd0, wsz}) - $signed({6'd0, hi}) : le + $signed({6'd0, a_lo});
                dx1  <= fx ? le + $signed({6'd0, wsz}) - 16'sd1 - $signed({6'd0, a_lo}) : le + $signed({6'd0, hi}) - 16'sd1;
                cell_raddr <= {dbuf, cbase + CELL_AW'(rcur * ncol) + CELL_AW'(c)};
                wst  <= W_KG;
            end
            W_KG: wst <= W_KH;                                   // cell_q valid next clock
            W_KH: begin : kh
                reg [15:0] bk;
                bk = dbank[{cell_q[13:11], 4'd0} +: 16];
                if (!cell_q[15] && a_hi > a_lo && dx0 <= cx1 && dx1 >= cx0) begin
                    jx0   <= (dx0 < cx0) ? cx0 : dx0;
                    jx1   <= (dx1 > cx1) ? cx1 : dx1;
                    jcode <= ({5'd0, cell_q[10:0]} | {bk[4:0], 11'd0}) + soff;
                    wst   <= W_KI;
                end else begin
                    a_lo <= a_hi;
                    c    <= c + 5'd1;
                    wst  <= (c + 5'd1 == ncol) ? W_NEXTS : W_KF;
                end
            end
            W_KI: begin
                // logical index of both span ends (flip: the screen runs backwards through the source)
                j0r <= fx ? 10'(le + $signed({6'd0, wsz}) - 16'sd1 - jx0) : 10'(jx0 - le);
                j1r <= fx ? 10'(le + $signed({6'd0, wsz}) - 16'sd1 - jx1) : 10'(jx1 - le);
                wst <= W_KQ;
            end
            W_KQ: begin
                kma <= {26'd0, j0r} * {11'd0, stepx};
                kmb <= {26'd0, j1r} * {11'd0, stepx};
                wst <= W_KJ;
            end
            W_KJ: begin
                ta_r <= {1'b0, kma} - {12'd0, c, 20'd0};         // position inside tile c
                tb_r <= {1'b0, kmb} - {12'd0, c, 20'd0};
                wst  <= W_KR;
            end
            W_KR: begin
                // clamped to [0, 16.0)
                mxa <= ta_r[36] ? 32'd0 : (ta_r[35:20] != 16'd0) ? 32'h000FFFFF : {12'd0, ta_r[19:0]};
                mxb <= tb_r[36] ? 32'd0 : (tb_r[35:20] != 16'd0) ? 32'h000FFFFF : {12'd0, tb_r[19:0]};
                jdx <= (stepx[24:21] != 4'd0) ? 21'h1FFFFF : stepx[20:0];
                a_lo <= a_hi;
                c    <= c + 5'd1;
                wst  <= W_CP;                                   // mxa/mxb valid in W_CP
            end

            W_NEXTS: begin
                n   <= n + 9'd1;
                wst <= (n + 9'd1 == dcount) ? W_DONE : W_SRD;
            end
            default: wst <= W_IDLE;
        endcase

        // ========================================================== fetcher
        begin : fetcher
            logic f_acc, f_use;
            logic [1:0] f_th;
            f_acc = mreq_valid && mreq_ready;
            f_use = rsp_mine && (f_disc == 2'd0);
            f_th = ftq[ftq_rd];
            if (f_acc) mreq_valid <= 1'b0;
            f_out <= f_out + (f_acc ? 2'd1 : 2'd0) - (rsp_mine ? 2'd1 : 2'd0);
            if (rsp_mine && f_disc != 2'd0) f_disc <= f_disc - 2'd1;
            // load: next job into the free slot at flp
            if (jf_pop) begin
                fsj[flp]     <= jf_head;
                fs_need[flp] <= jf_head[1:0];
                fs_full[flp] <= 1'b1;
                flp         <= !flp;
            end
            // issue: at most two reads at nb1_memory (posted + accepted)
            if (!mreq_valid && !abandon_now && (f_out - (rsp_mine ? 2'd1 : 2'd0)) < 2'd2 && fs_full[fip] && fs_need[fip] != 2'd0) begin
                mreq_valid  <= 1'b1;
                mreq_offset <= {1'b0, fsj[fip][JW-1 -: 16], fsj[fip][JW-17 -: 4], fs_need[fip][1] ? 4'd0 : 4'd8};
                fetch_cnt   <= fetch_cnt + 16'd1;
                ftq[ftq_wr]   <= {fip, !fs_need[fip][1]};
                ftq_wr       <= !ftq_wr;
                if (fs_need[fip][1]) begin
                    fs_need[fip][1] <= 1'b0; fs_wait[fip][1] <= 1'b1;
                    if (!fs_need[fip][0]) fip <= !fip;
                end else begin
                    fs_need[fip][0] <= 1'b0; fs_wait[fip][0] <= 1'b1;
                    fip <= !fip;
                end
            end
            // response: in issue order
            if (f_use) begin
                ftq_rd <= !ftq_rd;
                if (mrsp_err) mem_error <= 1'b1;
                if (f_th[0]) fsd[f_th[1]][63:0]   <= mrsp_err ? 64'hFFFF_FFFF_FFFF_FFFF : mrsp_line;
                else       fsd[f_th[1]][127:64] <= mrsp_err ? 64'hFFFF_FFFF_FFFF_FFFF : mrsp_line;
                fs_wait[f_th[1]][f_th[0] ? 0 : 1] <= 1'b0;
            end
            // hand-off to the writer (list order)
            if (wr_load) begin
                fs_full[fhp] <= 1'b0;
                fhp         <= !fhp;
            end
        end

        // ========================================================== writer
        if (wr_load) begin
            wr_busy <= 1'b1;
            // {code16, srow4, x0 10, x1 10, xi 21, step 21, fx, col4, pri4, halves2}
            wr_x    <= fsj[fhp][JW-21 -: 10];
            wr_x1   <= fsj[fhp][JW-31 -: 10];
            wr_xi   <= fsj[fhp][JW-41 -: 21];
            wr_step <= fsj[fhp][JW-62 -: 21];
            wr_fx   <= fsj[fhp][10];
            wr_col  <= fsj[fhp][9:6];
            wr_pri  <= fsj[fhp][5:2];
            wr_row  <= fsd[fhp];
            wr_slot <= rline[1:0];
            wr_mx   <= flx;
            wr_zc   <= zc;
            wr_a    <= flx ? 9'(10'd436 - fsj[fhp][JW-21 -: 10]) : fsj[fhp][JW-22 -: 9];   // x0 low 9 bits
        end else if (wr_busy && !dclr) begin
            if (wr_zc) begin
                // continuous: saturate at both ends (the span ends are clamped to the tile)
                if (wr_fx) wr_xi <= (wr_xi < wr_step) ? 21'd0 : wr_xi - wr_step;
                else       wr_xi <= ({1'b0, wr_xi} + {1'b0, wr_step} > 22'h1FFFFF) ? 21'h1FFFFF : wr_xi + wr_step;
            end else
                wr_xi <= wr_fx ? wr_xi - wr_step : wr_xi + wr_step;
            wr_x  <= wr_x + 10'd1;
            wr_a  <= wr_mx ? wr_a - 9'd1 : wr_a + 9'd1;
            if (wr_x == wr_x1) wr_busy <= 1'b0;
        end

        // ========================================================== lines
        if (line_done) begin
            line_active <= 1'b0;
            rline       <= rline + 8'd1;
        end
        if (!line_active && frame_on && rline < 8'd224 && {1'b0, rline} < ldisp + 9'd4 && !line_done) begin
            line_active <= 1'b1;
            n   <= 9'd0;
            wst <= (dcount == 9'd0) ? W_DONE : W_SRD;
        end
        if (line_end) begin
            if (vcount < 9'd224) ldisp <= vcount + 9'd1;
            // the display of line disp_next starts now: it must be complete
            if (frame_on && disp_next < 9'd224 && {1'b0, rline} == disp_next) begin
                if (drops != 16'hFFFF) drops <= drops + 16'd1;
                if (drops_cur != 8'hFF) drops_cur <= drops_cur + 8'd1;
                // M23: every read still owed to this line (posted or accepted) is dropped on return
                f_disc  <= f_out + (mreq_valid ? 2'd1 : 2'd0) - (rsp_mine ? 2'd1 : 2'd0);
                fs_full <= '0; flp <= 1'b0; fip <= 1'b0; fhp <= 1'b0; ftq_rd <= 1'b0; ftq_wr <= 1'b0;
                fs_need[0] <= '0; fs_need[1] <= '0; fs_wait[0] <= '0; fs_wait[1] <= '0;
                jf_rd <= '0; jf_wr <= '0; jf_cnt <= '0;
                wr_busy     <= 1'b0;
                wst         <= W_IDLE;
                line_active <= 1'b0;
                rline       <= rline + 8'd1;
            end
        end
        if (vblank_begin) begin
            // commit the buffer captured at the previous vblank, capture a new one
            dbuf     <= cbuf;
            dcount   <= count_s[cbuf];
            dbank    <= bank_s[cbuf];
            cell_count <= cells_s[cbuf];
            spr_count  <= count_s[cbuf];
            xs <= sext9(pos_in[31:16]) + 12'sd38;     // position word 1, + $26
            ys <= sext9(pos_in[15:0])  + 12'sd25;     // position word 0, + $19
            flx <= pos_in[32];                          // M20: position word 2 bit 0 (X), bit 1 (Y)
            zc  <= zoom_cont;                           // M21 experiment
            fly <= pos_in[33];
            fetch_frame <= fetch_cnt;
            fetch_cnt   <= '0;
            // per-frame drop statistics (the drawn frame ends here)
            if (drops_cur != 8'd0 && drop_frames != 16'hFFFF) drop_frames <= drop_frames + 16'd1;
            if (drops_cur > drop_max) drop_max <= drops_cur;
            drops_cur   <= '0;
            frame_on    <= 1'b1;
            rline       <= 8'd0;
            ldisp       <= 9'd0;
            line_active <= 1'b0;
            wst         <= W_IDLE;
            cbuf        <= !cbuf;
            bank_s[!cbuf] <= bank_in;
            cst   <= C_CLIP;
            cclip <= 6'd0;
            cph   <= 1'b0;
            cwait <= 1'b0;
        end

        if (reset) begin
            cst <= C_IDLE; wst <= W_IDLE;
            fs_full <= '0; flp <= 1'b0; fip <= 1'b0; fhp <= 1'b0; ftq_rd <= 1'b0; ftq_wr <= 1'b0;
            fs_need[0] <= '0; fs_need[1] <= '0; fs_wait[0] <= '0; fs_wait[1] <= '0; f_out <= '0; f_disc <= '0;
            frame_on <= 1'b0; line_active <= 1'b0;
            mreq_valid <= 1'b0;
            jf_rd <= '0; jf_wr <= '0; jf_cnt <= '0;
            wr_busy <= 1'b0;
            count_s[0] <= '0; count_s[1] <= '0; dcount <= '0;
            drops <= '0; overflow <= 1'b0; mem_error <= 1'b0;
            drops_cur <= '0; drop_frames <= '0; drop_max <= '0;
            fetch_cnt <= '0; fetch_frame <= '0; spr_count <= '0; cell_count <= '0;
        end
    end
endmodule
