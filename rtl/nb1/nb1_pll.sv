// Namco NB-1 MiSTer core -- system PLL.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// M1: one fractional-N PLL, 50 MHz board clock -> clk_sys = 96.768 MHz.
//
// 96.768 MHz = 2 x 48.384 MHz, the NB-1 master oscillator in MAME
// (48.384_MHz_XTAL in namconb1.cpp) [MAME-SOURCE]. Every NB-1 clock is an
// integer division of it, so nb1_clock_enables derives them as synchronous
// clock enables; no other fabric clock exists. [IMPLEMENTATION-DECISION]
//
// 96.768 / 50 = 1.93536 is not reachable with integer M/N/C counters inside
// the Cyclone V VCO range, hence fractional_vco_multiplier("true"). Quartus
// chooses M.K/N/C; the achieved frequency is reported in the fitter's PLL
// usage summary (see docs/M1_IMPLEMENTATION.md).
//
// HIERARCHY IS LOAD-BEARING: sys/sys_top.sdc puts the core clock in its own
// exclusive clock group only if it matches
//     *|pll|pll_inst|altera_pll_i|*[*].*|divclk
// i.e. emu instantiates this module as "pll", which contains "pll_inst",
// which contains the altera_pll as "altera_pll_i" -- the same shape as a
// MegaWizard-generated MiSTer PLL. With any other names clk_sys would be
// timed against every framework clock (seen in the first M1 fit as ~20 us of
// hold-fix routing).
module nb1_pll (
    input  wire refclk,   // CLK_50M
    input  wire rst,
    output wire clk_sys,  // 96.768 MHz
    output wire locked
);
    nb1_pll_core pll_inst (
        .refclk(refclk),
        .rst(rst),
        .clk_sys(clk_sys),
        .locked(locked)
    );
endmodule

module nb1_pll_core (
    input  wire refclk,
    input  wire rst,
    output wire clk_sys,
    output wire locked
);
    wire [0:0] clocks;

    altera_pll #(
        .fractional_vco_multiplier("true"),
        .reference_clock_frequency("50.0 MHz"),
        .operation_mode("direct"),
        .number_of_clocks(1),
        .output_clock_frequency0("96.768000 MHz"),
        .phase_shift0("0 ps"),
        .duty_cycle0(50),
        .pll_type("General"),
        .pll_subtype("General")
    ) altera_pll_i (
        .refclk(refclk),
        .rst(rst),
        .outclk(clocks),
        .locked(locked),
        .fboutclk(),
        .fbclk(1'b0)
    );

    assign clk_sys = clocks[0];
endmodule
