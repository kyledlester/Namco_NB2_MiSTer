// Namco NB-1 MiSTer core -- C123 tilemap control registers (M10).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// MAME maps NB-1 $660000-$66003F to namco_c123tmap control16_r/_w
// [MAME-SOURCE namconb1.cpp maincpu_am, namco_c123tmap.cpp]: 32 16-bit words,
// byte lanes merged with COMBINE_DATA, read back unchanged, no mirrors, and
// no device_reset (power-up zero only). Word meanings (docs/M10_RESEARCH.md):
//   w1, w5, w9, w13   layer 0-3 scroll X (bits 8-0); w1 bit 15 = global flip
//   w3, w7, w11, w15  layer 0-3 scroll Y (bits 8-0)
//   w16-w21           layer 0-5 priority (bits 2-0), bit 3 = layer disabled
//   w24-w29           layer 0-5 palette bank (bits 2-0): pen $1000 + bank*256
//   others            written by some games, no known effect (stored only)
// The side effects MAME applies at write time (tilemap enable, palette
// offset, scroll with the flip-time sign) are applied by the renderer from
// the stored words instead; see nb1_c123_render for the one MAME quirk this
// does not copy.
//
// CPU port: same one-cycle synchronous read as the other nb1_main_bus
// devices. Nebulas Ray never reads it (M5 captures); MAME returns the word.
module nb1_c123_ctl (
    input  wire         clk_sys,
    input  wire         access,
    input  wire         write,
    input  wire [5:1]   addr,       // byte offset $00-$3F, bit 0 implicit 0
    input  wire [15:0]  wdata,
    input  wire [1:0]   be,         // [1] even/MSB byte, [0] odd/LSB byte
    output reg  [15:0]  rdata = 16'h0000,
    output wire [511:0] regs_out,   // {w31, ..., w0}
    output reg  [31:0]  writes = '0 // CPU write cycles (diagnostic, never reset)
);
    reg [15:0] w [0:31];
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 32; i++) w[i] = 16'h0000;
`endif
// synthesis translate_on
    // Quartus: power-up zero for the registers (MAME std::fill 0)
    genvar g;
    generate for (g = 0; g < 32; g++) begin : g_w
        assign regs_out[g*16 +: 16] = w[g];
    end endgenerate

    always @(posedge clk_sys) if (access) begin
        if (write) begin
            if (be[1]) w[addr][15:8] <= wdata[15:8];
            if (be[0]) w[addr][7:0]  <= wdata[7:0];
            writes <= writes + 32'd1;
        end
        rdata <= w[addr];
    end
endmodule
