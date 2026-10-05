// Namco NB-1 MiSTer core -- CPU work/stack/shared RAM storage (M3).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Single-port 16-bit synchronous RAM with byte enables, one clk_sys cycle of
// read latency. Coded in the byte-lane pattern that Quartus 17.0 infers as
// M10K (packed [1:0][7:0] words, one write per lane) -- the pattern proven
// in the NA-1 core (na1_video_storage.sv, M16 note): bit-slice writes to a
// plain [15:0] array synthesised to registers and huge multiplexers there.
// Read-during-write data is never used by nb1_main_bus (no_rw_check).
// The array powers up as 0 (M10K default), like MAME's RAM. The real NB-1
// SRAM powers up undefined; Nebulas Ray clears/tests its RAM before use.
module nb1_cpu_ram #(parameter int WORDS = 8192, parameter int AW = 13) (
    input  wire          clk_sys,
    input  wire          en,        // access this cycle (read and/or write)
    input  wire          we,
    input  wire [AW-1:0] addr,      // word address
    input  wire [15:0]   wdata,
    input  wire [1:0]    be,        // [1] = D15-8 (even byte), [0] = D7-0
    output reg  [15:0]   rdata = '0
);
    (* ramstyle = "M10K, no_rw_check" *) reg [1:0][7:0] words[0:WORDS-1];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < WORDS; i++) words[i] = '0;
`endif
// synthesis translate_on
    always @(posedge clk_sys) if (en) begin
        if (we) begin
            if (be[1]) words[addr][1] <= wdata[15:8];
            if (be[0]) words[addr][0] <= wdata[7:0];
        end
        rdata <= words[addr];
    end
endmodule
