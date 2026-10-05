// Namco NB-1 MiSTer core -- CPU-facing raster events (M13).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// docs/M13_RESEARCH.md sections 4, 6 and 8: MAME draws tile line T with the
// C123/C116 state at the start of raster line T+1 (Nebulas Ray's
// update_to_line_before_posirq: the POS handler of line T has already run).
// The tile renderer (nb1_c123_render) latches line T's registers at the
// line_end that starts VIDEO line T-1. Both coincide when every event the CPU
// sees runs LEAD = 2 lines ahead of the video raster:
//
//   CPU line = video line + LEAD (mod 264)
//
// so the renderer's latch for line T happens at the start of CPU line T+1.
// Video timing, sync, the renderer, the C355 line walk and the video output
// stay on the video raster; only these CPU-side events move:
//
//   line_start  every line_end (a CPU line starts with every video line)
//   line_next   the CPU line that starts with it (POS compare, nb1_main_irq)
//   vbl         the start of CPU line 224 (= video line 224 - LEAD): the VBL
//               interrupt and the C355 position/bank latch
//   frame       the start of CPU line 0 (= video line 264 - LEAD): CPU release
//               and the CPU statistics frame
//
// All outputs are combinational from line_end and vcount (pulses coincide with
// line_end, exactly like vblank_begin/frame_end of nb1_video_timing).
module nb1_cpu_raster #(
    parameter integer V_TOTAL  = 264,
    parameter integer V_ACTIVE = 224,
    parameter integer LEAD     = 2
)(
    input  wire       line_end,       // nb1_video_timing
    input  wire [8:0] vcount,
    output wire       line_start,
    output wire [8:0] line_next,
    output wire       vbl,
    output wire       frame
);
    // video line that starts with this line_end, then + LEAD, mod V_TOTAL
    localparam [9:0] LEAD1 = LEAD + 1, VT = V_TOTAL;
    wire [9:0] vs = {1'b0, vcount} + LEAD1;
    wire [9:0] vw = (vs >= VT) ? (vs - VT) : vs;
    assign line_next  = vw[8:0];
    assign line_start = line_end;
    assign vbl        = line_end && (vcount == V_ACTIVE - 1 - LEAD);
    assign frame      = line_end && (vcount == V_TOTAL - 1 - LEAD);
endmodule
