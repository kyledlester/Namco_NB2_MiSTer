// Namco NB-2 MiSTer core -- pixel output: mixer (tiles only) + C116 output (M10).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// NB-2 copy of the NB-1 core's nb1_video_out (docs/PROVENANCE.md). NB-2 changes [MAME-SOURCE namconb1.cpp 600-630,
// namco_c169roz.cpp draw_helper]: the two C169 ROZ layers join the background, the C116 clip X offset is $4B, and
// the background is resolved with MAME's draw order: for pri 0..15, ROZ layers of that priority (layer 1, then 0),
// then (even pri) C123 layers of priority pri/2. Equivalently the opaque candidate with the largest key wins:
//   C123 entry (priority p, already the topmost C123 layer)  key {2p, 1, 0}, priority-map value 2p
//   ROZ layer L (priority P)                                 key {P, 0, 1-L}, priority-map value P
// pen: C123 $1000 + bank*256 + pixel; ROZ ($1800 + colour*256 + pixel) & $1FFF (colours 8-15 would be MAME's
// shadow pens; neither game uses them). The sprite mix compares the sprite priority with that map value
// (nothing opaque = 0), exactly as NB-1 compared it with the tile priority. The NB-1 description follows.
//
// Per displayed pixel [MAME-SOURCE namconb1.cpp screen_update, namco_c116.cpp]:
//   1. read the tile line-buffer entry at (vcount, hcount) (nb1_c123_render;
//      the entry is cleared behind the read)
//   2. mixer (C156 on the PCB) [MAME-SOURCE namconb1 sprite_mix_callback]:
//      the tile pen is $1000 + bank*256 + pixel of the topmost opaque tile
//      layer (priority p, 0 where none). The C355 line buffer (M11) holds at
//      most one sprite pixel (the last list entry drawn there), entry =
//      pri << 12 | pen, $FFFF = none. If sprite_pri >= p the sprite wins:
//      pen $FFE (palette 15, pixel $FE) is a shadow = tile pen | $800
//      (black stays black), any other pen is used as is ($000-$FFF).
//   3. C116 output: pen -> 8-bit R/G/B from the palette RAMs (video port of
//      nb1_c116_palette); black outside the C116 clip window
//        X + $4A >= reg0, X + $4A < reg1, Y + $21 >= reg2, Y + $21 < reg3
//      (16-bit signed registers, MAME's clip rectangle), and black where no
//      layer is opaque (MAME fills the bitmap with the black pen, RGB 0).
//      M13: the clip registers of line T are the ones latched with line T's
//      C123 state, i.e. at the line_end on which nb1_c123_render starts
//      drawing T (start of CPU line T+1, docs/M13_RESEARCH.md section 6),
//      one entry per line-buffer half. The VBL handler rewrites them at CPU
//      line 224 = video line 222, while video lines 222/223 are still to come.
//
// Timing: hcount/vcount/de change on the edge that ends a ce_pix cycle and
// then hold for 16 clk_sys (nb1_video_timing). Counting the first clk_sys
// of a pixel as A: the line-buffer read is issued in B, the entry captured at
// the end of C, the palette read in D and r/g/b registered at the end of E,
// so they are stable from A+5 to A+20, covering the ce_pix sample at A+15
// (and the 4 further registered overlay stages behind this module).
module nb2_video_out (
    input  wire         clk_sys,
    input  wire         ce_pix,
    input  wire [8:0]   hcount,
    input  wire [8:0]   vcount,
    input  wire         de,
    input  wire         line_end,       // M13: clip latch with the tile line
    input  wire         oflip,          // OSD 180 degrees: pixel (x, y) shows logical (287 - x, 223 - y)

    // nb1_c123_render display port
    output reg          disp_rd = 1'b0,
    output reg          disp_buf = 1'b0,
    output reg  [8:0]   disp_x = '0,
    input  wire [15:0]  disp_entry,

    // nb2_roz_render display port (read with the tile port, same timing)
    input  wire [16:0]  roz_e0,
    input  wire [16:0]  roz_e1,

    // nb1_c355_render display port (M11)
    output reg          spr_rd = 1'b0,
    output reg  [1:0]   spr_slot = '0,
    output reg  [8:0]   spr_x = '0,
    input  wire [15:0]  spr_entry,

    // nb1_c116_palette video port
    output wire [12:0]  vid_pen,
    input  wire [23:0]  vid_rgb,
    input  wire [127:0] c116_regs,

    output reg  [7:0]   r = 8'd0,
    output reg  [7:0]   g = 8'd0,
    output reg  [7:0]   b = 8'd0
);
    // c0: first clk_sys of the pixel
    reg ps = 1'b0;
    always @(posedge clk_sys) ps <= ce_pix;

    reg        de1 = 1'b0, de2 = 1'b0, p1 = 1'b0, p2 = 1'b0;
    reg [15:0] e1 = '0, s1 = 16'hFFFF;
    reg [16:0] r0 = '0, r1 = '0;
    reg        clip2 = 1'b0;
    reg [8:0]  x0 = '0, y0 = '0;

    // M13: clip registers 0-3 per tile line (same line numbering as the
    // renderer: the line_end that starts video line D latches line D+1, i.e.
    // line vcount+2). That target has the parity of vcount (also across the
    // 263 -> 0 wrap) and is a visible line (< 224) exactly when vcount < 222
    // or vcount >= 262. The condition is registered during the line (vcount is
    // stable for the whole line), so nothing combinational hangs off vcount at
    // line_end (timing: no logic shared with the renderers' own compares).
    reg         cl_ok = 1'b0;
    always @(posedge clk_sys) cl_ok <= (vcount < 9'd222) || (vcount >= 9'd262);
    // two registers (not an array: Quartus would infer an M10K for it)
    reg  [63:0] clip_ln0 = '0, clip_ln1 = '0;
    always @(posedge clk_sys)
        if (line_end && cl_ok) begin
            if (vcount[0]) clip_ln1 <= c116_regs[63:0];
            else           clip_ln0 <= c116_regs[63:0];
        end
    wire [63:0] clip = y0[0] ? clip_ln1 : clip_ln0;

    // C116 window, 17-bit signed compares against the signed 16-bit registers
    // x0 is the logical x (mirrored at the read when flipped); y0 is the display line (it selects the clip
    // latch), the logical line is 223 - y0 when flipped
    wire [8:0]  ly = oflip ? 9'd223 - y0 : y0;
    wire signed [16:0] hx = $signed({8'd0, x0}) + 17'sd75;
    wire signed [16:0] vy = $signed({8'd0, ly}) + 17'sd33;
    wire signed [16:0] c0 = $signed({clip[15], clip[15:0]});
    wire signed [16:0] c1 = $signed({clip[31], clip[31:16]});
    wire signed [16:0] c2 = $signed({clip[47], clip[47:32]});
    wire signed [16:0] c3 = $signed({clip[63], clip[63:48]});
    wire in_win = (hx >= c0) && (hx < c1) && (vy >= c2) && (vy < c3);

    // mixer
    wire        c_ok  = e1[15];
    wire [5:0]  c_key = {e1[13:11], 1'b0, 1'b1, 1'b0};               // {2p, stage 1, 0}
    wire        z0_ok = r0[16], z1_ok = r1[16];
    wire [5:0]  z0_key = {r0[15:12], 1'b0, 1'b1};                    // {P, stage 0, order 1}
    wire [5:0]  z1_key = {r1[15:12], 1'b0, 1'b0};
    // winner among the opaque candidates
    wire        w01   = z0_ok && (!z1_ok || z0_key > z1_key);        // ROZ 0 beats ROZ 1
    wire        z_ok  = z0_ok || z1_ok;
    wire [5:0]  z_key = w01 ? z0_key : z1_key;
    wire [16:0] z_e   = w01 ? r0 : r1;
    wire        use_c = c_ok && (!z_ok || c_key > z_key);
    wire        t_ok  = c_ok || z_ok;
    wire [3:0]  t_pri = !t_ok ? 4'd0 : use_c ? {e1[13:11], 1'b0} : z_e[15:12];
    wire [12:0] t_pen = use_c ? {1'b1, 1'b0, e1[10:0]} : (13'h1800 + {1'b0, z_e[11:0]});
    wire        s_ok  = (s1 != 16'hFFFF) && (s1[15:12] >= t_pri);
    wire        s_shd = s1[11:0] == 12'hFFE;
    wire        m_ok  = s_ok ? (!s_shd || t_ok) : t_ok;
    assign vid_pen = !s_ok ? t_pen : s_shd ? (t_pen | 13'h0800) : {1'b0, s1[11:0]};

    // read x: the line buffers hold logical x 0..287; flipped, display x reads logical 287 - x (the same set of
    // entries per line, so the clear-behind-read is unchanged)
    wire [8:0] rx = oflip ? 9'd287 - hcount : hcount;

    always @(posedge clk_sys) begin
        // c0: request the entry of the pixel now on the raster
        disp_rd  <= 1'b0;
        spr_rd   <= 1'b0;
        if (ps) begin
            spr_rd   <= de;
            spr_x    <= rx;
            spr_slot <= vcount[1:0];
            disp_rd  <= de;
            disp_x   <= rx;
            disp_buf <= vcount[0];
            x0       <= rx;
            y0       <= vcount;
        end
        // c1 (read issued), c2 (entry valid): pipeline flags
        p1  <= ps;
        de1 <= de;
        p2  <= p1;
        if (p2) begin
            e1    <= disp_entry;      // entries valid now; palette address from them next
            r0    <= roz_e0;
            r1    <= roz_e1;
            s1    <= spr_entry;
            de2   <= de1;
            clip2 <= in_win;
        end
    end

    // c3: palette data for e1 valid one clk_sys after vid_pen settles
    reg p3 = 1'b0, p4 = 1'b0;
    always @(posedge clk_sys) begin
        p3 <= p2;
        p4 <= p3;
        if (p4) {r, g, b} <= (de2 && clip2 && m_ok) ? vid_rgb : 24'h000000;
    end
endmodule
