// Namco NB-1 MiSTer core -- clock-enable generator.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// clk_sys = 96.768 MHz = 2 x the 48.384 MHz NB-1 master oscillator.
// All NB-1 clocks are exact integer divisions [MAME-SOURCE, namconb1.cpp]:
//
//   enable     NB-1 clock                    divide  pulses per 48 clk_sys
//   ce_xtal    48.384 MHz master (reference)   /2      24
//   ce_cpu     68EC020  24.192 MHz (XTAL/2)    /4      12
//   ce_c352    C352     24.192 MHz (XTAL/2)    /4      12  (same cadence as ce_cpu)
//   ce_c75     C75 input 16.128 MHz (XTAL/3)   /6       8
//   ce_pix     pixel     6.048 MHz (XTAL/8)    /16      3
//
// One 48-cycle phase counter (48 = lcm(2,4,6,16)) drives every enable, so
// their relative phase is fixed forever. Each enable is a single clk_sys
// cycle wide and fires on the LAST cycle of its period; at phase 47 all of
// them coincide, and phase 0 is the first cycle of a new common period.
//
// The phase counter is initialised at configuration and free-runs; it is
// deliberately NOT reset. A reset that re-phased the enables would make the
// free-running raster (nb1_video_timing) glitch. Determinism comes from the
// power-up value, and resets release only at phase 0 (nb1_reset).
// ce_cpu is the NOMINAL 68EC020 time base, not a hard ceiling. Per standing
// rule SR-2 (docs/AGENT_HANDOFF.md section 7), the future CPU wrapper may
// schedule catch-up cycles after memory stalls so that SDRAM latency does
// not lower the effective CPU rate.
// The C352 84 kHz sample tick (/1152) is not generated in M1: its rate
// is still an open question (NB1_HARDWARE_SPEC U17).
module nb1_clock_enables (
    input  wire       clk_sys,
    output reg        ce_xtal = 1'b0,
    output reg        ce_cpu  = 1'b0,
    output reg        ce_c352 = 1'b0,
    output reg        ce_c75  = 1'b0,
    output reg        ce_pix  = 1'b0,
    output reg  [5:0] phase   = 6'd0   // 0..47, current cycle of the common period
);
    localparam [5:0] PERIOD = 6'd48;

    // The mod-6 position is kept by its own counter. It wraps together with
    // `phase` because 48 is a multiple of 6.
    reg  [2:0] mod6 = 3'd0;

    wire [5:0] phase_next = (phase == PERIOD - 6'd1) ? 6'd0 : phase + 6'd1;
    wire [2:0] mod6_next  = (mod6 == 3'd5) ? 3'd0 : mod6 + 3'd1;

    always @(posedge clk_sys) begin
        phase   <= phase_next;
        mod6    <= mod6_next;
        // Registered decode of the NEXT phase: ce_* is high during the cycle
        // whose phase is the last one of that enable's period.
        ce_xtal <= (phase_next[0]   == 1'b1);     // phase % 2  == 1
        ce_cpu  <= (phase_next[1:0] == 2'd3);     // phase % 4  == 3
        ce_c352 <= (phase_next[1:0] == 2'd3);
        ce_c75  <= (mod6_next       == 3'd5);     // phase % 6  == 5
        ce_pix  <= (phase_next[3:0] == 4'd15);    // phase % 16 == 15
    end
endmodule
