// Namco NB-1 MiSTer core -- CRT Adjust glue (M17).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// OSD CRT Adjust for the native 15-kHz picture: the owner's NA-1/NA-2 integration (NA1.sv, M26) ported
// to the NB-1 raster, around the UNMODIFIED upstream rtl/vendor/crt_adjust.sv (MiSTer-CRT-Adjust,
// rmonic79). Same OSD controls, status bits, encodings, ranges and defaults as NA-1/NA-2:
//   [96]      CRT Adjust Off / On (0 = Off = default = TRUE bypass: the native stream, zero latency)
//   [116:112] H-Size OSD index -> -12..+10, one step = 1 % (index 0 = 0, 1..10 = +1..+10,
//             11..22 = -12..-1, 23..31 unreachable = 0); + = WIDER
//   [104:101] H-Position signed -8..+7, one step = 6 pixels (the module's HPOS_SYNCSHIFT)
//   [108:105] V-Shift    signed -8..+7, one step = 1 line
// All-zero status = Off and neutral, so an untouched user gets the M16 picture exactly.
//
// Presentation only. The module sits after the NB-1 raster (nb1_video_timing) and the diagnostic
// overlay: it moves/resizes the CONTENT through a line buffer while the line period, frame period,
// HSync/VSync widths and the pixel clock stay native. Nothing upstream (line_end / frame_end, the M13
// CPU raster, POS, the C123/C355 renderers, CPU, C75, C352, SDRAM) sees it.
//
// As NA-1/NA-2:
//   - the controls are sampled once per frame (frame_event, inside vertical blanking): a mid-frame
//     change disturbs the line-buffer engine for that frame, so each frame is internally consistent;
//   - the adjust is gated off while the scandoubler is active (Fx != None or forced_scandoubler): the
//     module's read rate is built for the native 15-kHz pixel rate;
//   - the read-rate generator restarts on the module's hs_ref_out;
//   - H-Position direction as NA-1/NA-2 on hardware (+ moves the picture LEFT, measured on NA's
//     HPOS_SYNCSHIFT engine by sim/m17_crt_tb.sv); V-Shift is the module's, unchanged;
//   - read CE: the real pixel CE while H-Size is 0 (byte-exact), else an exact rational NCO
//         READ_INC = PIXEL_HZ - hsize * STEP,  STEP = PIXEL_HZ / 100
//     NB-1: PIXEL_HZ = clk_sys / 16 = 6,048,000, STEP = 60,480 (exactly 1.000 %). Max READ_INC
//     (hsize -12) 6,773,760; phase + inc < 96,768,000 + 6,773,760 < 2^27. Min (hsize +10) 5,443,200.
//   - the adjusted stream also reaches HDMI while CRT Adjust is On (core-side insertion; isolating HDMI
//     would need sys/sys_top.v edits). Off leaves HDMI exactly as M16.
//
// NB-1-safe engine (owner decision, docs/M17_IMPLEMENTATION.md). The NB-1 line has 96 blanking pixels
// (NA-1/NA-2: 152) and HSync right after the active area (native line: active 0..287, HSync 310..341),
// so NA's exact engine (left-anchored H-Size, HPOS_SYNCSHIFT) corrupts the right edge at H-Size +6..+10,
// loses the picture at H-Position -24..-48 and drops the first and last lines (sim/m17_crt_tb.sv).
// Here, with the same OSD:
//   - HPOS_CONTENTSHIFT (1): HSync out is the native HSync (48 pixels later, below) in every setting;
//   - the module gets HSync/VSync 48 pixels late (SYNC_LAG), i.e. its content already 48 pixels
//     early, and the content offset + 48: the offsets it sees are never negative. (Upstream
//     crt_adjust.sv zero-extends a negative CONTENTSHIFT offset -- `hoff_s` is a ?: with an unsigned
//     arm -- and the picture disappears; NA-1/NA-2 uses SYNCSHIFT and never meets it. The vendored
//     file stays unmodified.) The geometry against the output sync is unchanged; the whole output
//     is 48 pixels (7.9 us) later than the native raster, which a CRT cannot see;
//   - H-Size scales about the picture centre (an automatic content offset per value), so all of
//     -12..+10 is shown whole at H-Position 0;
//   - the total content offset (centring + H-Position) is clamped to the blanking: the picture never
//     overlaps the HSync pulse or runs past the next one (past the limit it stops moving);
//   - the module's line window runs HSync to HSync (it starts at native pixel 310, holding the next
//     line's active pixels), so its vertical blank input is the blank of the line it is WRITING, and
//     the vertical blank sent on is the blank of the line it is READING (the module's own vb_out is
//     the native one, a line early for the delayed content): all 224 lines are shown;
//   - V-Shift is clamped to -6..+7 so VSync (line 231, 3 lines) never lands on a picture line (the
//     content is one line late through the buffer: native lines 1..224).
module nb1_crt_adjust #(
    parameter integer SYS_HZ = 96_768_000,
    parameter integer PIX_DIV = 16,
    parameter integer HTOTAL = 384,
    parameter integer VTOTAL = 264
) (
    input  wire        clk_sys,
    input  wire        ce_pix,
    input  wire        frame_event,     // one pulse per frame, inside vertical blanking
    // OSD
    input  wire        osd_on,          // status[96]
    input  wire [4:0]  osd_hsize,       // status[116:112]
    input  wire [3:0]  osd_hpos,        // status[104:101]
    input  wire [3:0]  osd_vshift,      // status[108:105]
    input  wire        sd_off,          // scandoubler off (Fx None and not forced)
    input  wire        vb_next,         // the vertical blank of the NEXT native line (see above)
    // native stream
    input  wire [23:0] rgb_in,
    input  wire        hblank_in,
    input  wire        vblank_in,
    input  wire        hsync_in,
    input  wire        vsync_in,
    // to arcade_video
    output wire        ce_out,
    output wire [23:0] rgb_out,
    output wire        hblank_out,
    output wire        vblank_out,
    output wire        hsync_out,
    output wire        vsync_out,
    // state (bench / overlay)
    output wire        active,
    output reg  signed [4:0] hsize_s = 5'sd0,
    output reg  signed [3:0] hpos_s  = 4'sd0,
    output reg  signed [3:0] vsh_s   = 4'sd0
);
    reg crt_on = 1'b0;
    always @(posedge clk_sys) if (frame_event) begin
        crt_on  <= osd_on;
        hsize_s <= (osd_hsize <= 5'd10) ? $signed(osd_hsize)
                 : (osd_hsize <= 5'd22) ? $signed(osd_hsize - 5'd23)
                 : 5'sd0;
        hpos_s  <= $signed(osd_hpos);
        vsh_s   <= $signed(osd_vshift);
    end
    assign active = crt_on && sd_off;

    // sign-extended explicitly (a bare size cast of a signed value evaluates unsigned)
    // content offset (module: > 0 = content right) = centring for H-Size - 6 * H-Position, clamped to
    // [lo, hi]: lo = first pixel right after the HSync pulse, hi = last pixel before the next HSync
    // (in read ticks: hc = round(218 f) - 218, lo = ceil(32 f) - 74, hi = floor(384 f) - 365 with
    // f = 1 - hsize / 100; active window 74..361 of the HSync-to-HSync line, centre 218)
    reg signed [8:0] hc, lo, hi;
    always_comb begin
        case (hsize_s)
            -5'sd12: begin hc = 26; lo = -38; hi = 65; end
            -5'sd11: begin hc = 24; lo = -38; hi = 61; end
            -5'sd10: begin hc = 22; lo = -38; hi = 57; end
            -5'sd9: begin hc = 20; lo = -39; hi = 53; end
            -5'sd8: begin hc = 17; lo = -39; hi = 49; end
            -5'sd7: begin hc = 15; lo = -39; hi = 45; end
            -5'sd6: begin hc = 13; lo = -40; hi = 42; end
            -5'sd5: begin hc = 11; lo = -40; hi = 38; end
            -5'sd4: begin hc = 9; lo = -40; hi = 34; end
            -5'sd3: begin hc = 7; lo = -41; hi = 30; end
            -5'sd2: begin hc = 4; lo = -41; hi = 26; end
            -5'sd1: begin hc = 2; lo = -41; hi = 22; end
            5'sd0: begin hc = 0; lo = -42; hi = 19; end
            5'sd1: begin hc = -2; lo = -42; hi = 15; end
            5'sd2: begin hc = -4; lo = -42; hi = 11; end
            5'sd3: begin hc = -7; lo = -42; hi = 7; end
            5'sd4: begin hc = -9; lo = -43; hi = 3; end
            5'sd5: begin hc = -11; lo = -43; hi = -1; end
            5'sd6: begin hc = -13; lo = -43; hi = -5; end
            5'sd7: begin hc = -15; lo = -44; hi = -8; end
            5'sd8: begin hc = -17; lo = -44; hi = -12; end
            5'sd9: begin hc = -20; lo = -44; hi = -16; end
            5'sd10: begin hc = -22; lo = -45; hi = -20; end
            default: begin hc = 0; lo = -42; hi = 21; end
        endcase
    end
    reg signed [8:0] hoff_q = 9'sd0;
    wire signed [9:0] hoff_raw = $signed({hc[8], hc}) - ($signed({{6{hpos_s[3]}}, hpos_s}) * 10'sd6);
    always @(posedge clk_sys)
        hoff_q <= (hoff_raw < $signed({lo[8], lo})) ? lo : (hoff_raw > $signed({hi[8], hi})) ? hi : hoff_raw[8:0];
    localparam integer SYNC_LAG = 48;
    wire signed [8:0] hoffset = active ? (hoff_q + 9'sd48) : 9'sd0;
    wire signed [3:0] vsh_c   = (vsh_s < -4'sd6) ? -4'sd6 : vsh_s;
    wire signed [5:0] voffset = active ? $signed({{2{vsh_c[3]}}, vsh_c}) : 6'sd0;

    localparam integer PIXEL_HZ = SYS_HZ / PIX_DIV;
    localparam integer STEP     = (PIXEL_HZ + 50) / 100;
    wire hs_ref;
    reg  hs_ref_d = 1'b0;
    always @(posedge clk_sys) hs_ref_d <= hs_ref;
    wire hs_ref_rise = hs_ref && !hs_ref_d;
    // registered: hsize_s changes only at a frame event, so one clock late is exact
    reg  signed [31:0] read_inc = PIXEL_HZ;
    always @(posedge clk_sys) read_inc <= PIXEL_HZ - (hsize_s * STEP);
    reg  [26:0] phase = 27'd0;
    wire [27:0] phase_sum = {1'b0, phase} + read_inc[26:0];
    wire rd_tick = (phase_sum >= SYS_HZ);
    always @(posedge clk_sys) begin
        if (hs_ref_rise)  phase <= 27'd0;
        else if (rd_tick) phase <= phase_sum - SYS_HZ;
        else              phase <= phase_sum[26:0];
    end
    reg use_nco = 1'b0;
    always @(posedge clk_sys) use_nco <= active && (hsize_s != 5'sd0);
    wire rd_ce = use_nco ? rd_tick : ce_pix;

    // vertical blank of the line being written in the module's HSync-to-HSync window: after the native
    // HSync (pixel 310) that window holds the NEXT line; sampled by the module at the HSync rise
    wire vb_wr = vb_next;
    // HSync / VSync, SYNC_LAG pixels late (native pixel CE)
    reg [SYNC_LAG-1:0] hs_dl = '0, vs_dl = '0;
    always @(posedge clk_sys) if (ce_pix) begin
        hs_dl <= {hs_dl[SYNC_LAG-2:0], hsync_in};
        vs_dl <= {vs_dl[SYNC_LAG-2:0], vsync_in};
    end
    wire hs_lag = hs_dl[SYNC_LAG-1];
    wire vs_lag = vs_dl[SYNC_LAG-1];
    // ... and of the line being read (the module's vb_line / vb_active pair, same edge)
    reg hs_in_d = 1'b0, vb_l1 = 1'b0, vb_rd = 1'b0;
    always @(posedge clk_sys) if (ce_pix) begin
        hs_in_d <= hs_lag;
        if (hs_lag && !hs_in_d) begin vb_l1 <= vb_wr; vb_rd <= vb_l1; end
    end
    wire [23:0] a_rgb;
    wire a_hs, a_vs, a_hb, a_vb;
    crt_adjust #(.VTOTAL(VTOTAL), .HTOTAL(HTOTAL), .HPOS_MODE(1)) crt_adjust (
        .clk(clk_sys), .pxl_cen(ce_pix), .pxl2_cen(rd_ce),
        .active(active),
        .hsize(hsize_s), .hoffset(hoffset), .voffset(voffset),
        .r_in(rgb_in[23:16]), .g_in(rgb_in[15:8]), .b_in(rgb_in[7:0]),
        .hs_in(hs_lag), .vs_in(vs_lag), .hb_in(hblank_in), .vb_in(vb_wr),
        .r_out(a_rgb[23:16]), .g_out(a_rgb[15:8]), .b_out(a_rgb[7:0]),
        .hs_out(a_hs), .vs_out(a_vs), .hb_out(a_hb), .vb_out(a_vb),
        .hs_ref_out(hs_ref));

    // TRUE bypass when Off: the native stream, zero added latency (the M16 picture)
    assign ce_out     = active ? rd_ce : ce_pix;
    assign rgb_out    = active ? a_rgb : rgb_in;
    assign hblank_out = active ? a_hb  : hblank_in;
    assign vblank_out = active ? vb_rd : vblank_in;
    assign hsync_out  = active ? a_hs  : hsync_in;
    assign vsync_out  = active ? a_vs  : vsync_in;
endmodule
