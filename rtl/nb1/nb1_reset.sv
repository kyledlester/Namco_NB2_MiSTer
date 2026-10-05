// Namco NB-1 MiSTer core -- reset generation.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Two layers:
//
// 1. reset_sys: asserted asynchronously by any reset source (MiSTer RESET,
//    OSD reset, user button, PLL not locked), released synchronously to
//    clk_sys through a 3-flop synchroniser. It is for control/infrastructure
//    logic (future loader, OSD-facing logic). This is the proven NA-1 style.
//
// 2. Device resets (reset_cpu, reset_c75, reset_video): set whenever
//    reset_sys is high. Released only on the clk_sys edge that starts a new
//    frame (frame_end from nb1_video_timing, which always falls on
//    clock-enable phase 47 -> next cycle is phase 0, raster (0,0)).
//    Emulated NB-1 hardware therefore always starts at the same raster
//    position and clock-enable phase, like MAME's power-on at vpos 0. That
//    makes FPGA runs reproducible and comparable with MAME frame-numbered
//    traces. Cost: up to one frame (16.76 ms) of extra reset. [IMPLEMENTATION-DECISION]
//
//    NB-1 specifics are deliberately NOT modelled here. The C75's run/halt
//    from cpureg $400018 and the 68020 RESET instruction belong to later
//    milestones and will be ANDed in by their owners. The three device
//    outputs are identical in M1 and exist so those domains have a clean,
//    separately routed reset to start from.
//
// Startup: every register has a configuration-time value, so reset_sys and
// all device resets are asserted from the first clk_sys edge after
// configuration, without depending on RESET behaviour.
module nb1_reset (
    input  wire clk_sys,
    input  wire reset_request,   // async-safe: OR of all reset sources (active high)
    input  wire pll_locked,
    input  wire frame_end,       // from nb1_video_timing
    output wire reset_sys,
    output reg  reset_cpu   = 1'b1,
    output reg  reset_c75   = 1'b1,
    output reg  reset_video = 1'b1
);
    wire reset_async = reset_request | ~pll_locked;

    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *)
    reg [2:0] sync = 3'b111;
    always @(posedge clk_sys or posedge reset_async) begin
        if (reset_async) sync <= 3'b111;
        else             sync <= {sync[1:0], 1'b0};
    end
    assign reset_sys = sync[2];

    // Asynchronous assertion (follows reset_sys immediately, even if clk_sys
    // stops while the PLL is unlocked); synchronous, frame-aligned release.
    always @(posedge clk_sys or posedge reset_sys) begin
        if (reset_sys) begin
            reset_cpu   <= 1'b1;
            reset_c75   <= 1'b1;
            reset_video <= 1'b1;
        end else if (frame_end) begin
            reset_cpu   <= 1'b0;
            reset_c75   <= 1'b0;
            reset_video <= 1'b0;
        end
    end
endmodule
