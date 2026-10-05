// Namco NB-1 MiSTer core -- raster timing generator.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Counter frame: MAME's namconb1 screen, set_raw(48.384 MHz/8, 384, 0, 288, 264, 0, 224):
//   hcount 0..383 per line, hcount 0..287 active     [MAME-SOURCE]
//   vcount 0..263 per frame, vcount 0..223 active    [MAME-SOURCE]
//   pixel 6.048 MHz, 15.750 kHz line, 59.659 Hz frame [MAME-CONFIRMED -listxml]
//   The VBLANK IRQ is raised at vcount 224, the start of vblank [MAME-SOURCE scantimer]
// Keeping MAME's numbering lets later bring-up compare beam positions with
// MAME traces directly.
//
// C116 counter frame [INFERRED]: Nebulas Ray programs the C116 display
// window to H $004A..$0169 and V $0021..$0100. MAME maps those to x 0..287 and
// y 0..223 by subtracting $4A and $21 (namconb1_state::screen_update). So the
// C116's own counters read hcount+74 and vcount+33, modulo the totals. They
// are exported as c116_h/c116_v for the later C116 window/POSIRQ logic.
//
// Sync placement: NOT established by MAME or any document we have (MAME gives
// only totals and active bounds). [UNKNOWN] on the PCB.
//   Start: [INFERRED, weak] assume the C116 counters' zero is the sync start.
//          HSYNC starts at c116_h == 0, i.e. hcount 384-74 = 310.
//          VSYNC starts at c116_v == 0, i.e. vcount 264-33 = 231.
//          This gives front porches of 22 px / 7 lines.
//   Width: [IMPLEMENTATION-DECISION] HSYNC 32 px (5.29 us), VSYNC 3 lines,
//          typical 15 kHz arcade values; back porches 42 px / 30 lines.
//   All four numbers are parameters; change them here only with evidence.
//
// The raster free-runs from configuration and is never reset, so HDMI/CRT
// sync stays stable through resets (same principle as the NA-1 core).
// Events for future devices are gated by the consumers' own resets.
//
// Output timing: every output is registered and changes on the clk_sys edge
// that ends a ce_pix cycle. Outputs therefore describe the "current" pixel
// for its whole 16-cycle lifetime, including the ce_pix cycle in which
// MiSTer's CE_PIXEL samples it.
module nb1_video_timing #(
    parameter integer H_TOTAL      = 384,
    parameter integer H_ACTIVE     = 288,
    parameter integer V_TOTAL      = 264,
    parameter integer V_ACTIVE     = 224,
    parameter integer HSYNC_START  = 310,
    parameter integer HSYNC_WIDTH  = 32,
    parameter integer VSYNC_START  = 231,
    parameter integer VSYNC_LINES  = 3,
    parameter integer C116_H_OFFS  = 74,   // $4A
    parameter integer C116_V_OFFS  = 33    // $21
)(
    input  wire       clk_sys,
    input  wire       ce_pix,
    output reg  [8:0] hcount = 9'd0,
    output reg  [8:0] vcount = 9'd0,
    output reg  [8:0] c116_h = C116_H_OFFS,
    output reg  [8:0] c116_v = C116_V_OFFS,
    output reg        hblank = 1'b0,
    output reg        vblank = 1'b0,
    output reg        de     = 1'b1,
    output reg        hsync  = 1'b0,   // active high
    output reg        vsync  = 1'b0,   // active high
    // Single clk_sys pulses, asserted in the ce_pix cycle BEFORE the edge that
    // moves the raster onto (0, v) or (0, 0). After that edge hcount == 0.
    output wire       line_end,
    output wire       frame_end,
    // Single clk_sys pulse in the ce_pix cycle before vcount becomes V_ACTIVE
    // (the MAME VBLANK IRQ point), for future interrupt logic.
    output wire       vblank_begin
);
    wire last_h = (hcount == H_TOTAL - 1);
    wire last_v = (vcount == V_TOTAL - 1);

    wire [8:0] h_next = last_h ? 9'd0 : hcount + 9'd1;
    wire [8:0] v_next = last_h ? (last_v ? 9'd0 : vcount + 9'd1) : vcount;

    assign line_end     = ce_pix & last_h;
    assign frame_end    = ce_pix & last_h & last_v;
    assign vblank_begin = ce_pix & last_h & (vcount == V_ACTIVE - 1);

    function automatic [8:0] wrap_add(input [8:0] a, input integer offs, input integer total);
        integer s;
        begin
            s = a + offs;
            if (s >= total) s = s - total;
            wrap_add = s[8:0];
        end
    endfunction

    always @(posedge clk_sys) begin
        if (ce_pix) begin
            hcount <= h_next;
            vcount <= v_next;
            c116_h <= wrap_add(h_next, C116_H_OFFS, H_TOTAL);
            c116_v <= wrap_add(v_next, C116_V_OFFS, V_TOTAL);
            hblank <= (h_next >= H_ACTIVE);
            vblank <= (v_next >= V_ACTIVE);
            de     <= (h_next < H_ACTIVE) && (v_next < V_ACTIVE);
            hsync  <= (h_next >= HSYNC_START) && (h_next < HSYNC_START + HSYNC_WIDTH);
            // VSYNC changes at the start of a line (hcount 0).
            vsync  <= (v_next >= VSYNC_START) && (v_next < VSYNC_START + VSYNC_LINES);
        end
    end
endmodule
