// Namco NB-1 MiSTer core -- C75 external interrupt sources INT0 / INT2 (M8).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// [MAME-SOURCE namconb1.cpp] The C75's INT0 and INT2 come from two periodic
// 60 Hz timers ("mcu_irq0" / "mcu_irq2", HOLD_LINE), started with the
// machine and never reset, both firing at the same instants. MAME marks the
// source "TODO: Real sources of these" and keeps 60 Hz because "has to be
// 60 hz or music will go crazy in nebulray, vshoot, gslugrs*". The physical
// source is [UNKNOWN] (NB1_HARDWARE_SPEC U2).
//
// [IMPLEMENTATION, provisional, docs/M8_RESEARCH.md section 6] This module
// reproduces MAME: one free-running counter of exactly 96.768 MHz / 60 =
// 1,612,800 clk_sys, not reset by anything (like MAME's timers it runs from
// configuration), pulsing INT0 and INT2 in the same cycle. The pulses set the
// M37702's interrupt requests, which stay pending until the CPU takes them
// (the SFR block's pend/int_ctl bit 3), matching HOLD_LINE.
// The genuine BIOS depends on it: the boot handshake the 68020 waits for is
// written only after the BIOS has taken INT0/INT2 (docs/M8_RESEARCH.md
// sections 5-6).
module nb1_c75_irq #(
    parameter int PERIOD = 1612800           // clk_sys per tick (60.000 Hz)
) (
    input  wire clk_sys,
    output reg  irq0 = 1'b0,
    output reg  irq2 = 1'b0,
    output reg  [7:0] ticks = '0             // diagnostic: wraps every 256 ticks
);
    reg [$clog2(PERIOD)-1:0] cnt = '0;
    always @(posedge clk_sys) begin
        irq0 <= 1'b0;
        irq2 <= 1'b0;
        if (cnt == PERIOD - 1) begin
            cnt   <= '0;
            irq0  <= 1'b1;
            irq2  <= 1'b1;
            ticks <= ticks + 8'd1;
        end else
            cnt <= cnt + 1'b1;
    end
endmodule
