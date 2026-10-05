// Namco NB-1 MiSTer core -- C123 tilemap VRAM (M5 CPU port, M10 video port).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// MAME maps NB-1 $640000-$64FFFF to namco_c123tmap videoram16_r/_w
// [MAME-SOURCE namconb1.cpp maincpu_am, namco_c123tmap.cpp]: a flat array of
// 32,768 16-bit words, word index = A15:A1, no mirrors inside the window, no
// CPU-visible side effects, byte lanes merged with COMBINE_DATA. The 68EC020
// reaches it through the 16-bit big-endian TG68K cycle: D15:8 is the even
// byte, D7:0 the odd byte; longwords arrive as two word cycles.
//
// The whole 64 KiB is CPU RAM, not only the tilemap area: Nebulas Ray keeps
// its self-test state block (and later its stack) at $64FE80+
// (docs/M5_RESEARCH.md section 4).
//
// Storage is not cleared by reset: MAME's C123 has no device_reset and the
// PCB VRAM is external SRAM. Nebulas Ray relies on this across a warm reset
// (it skips its memory tests when the magic at $64FE82 survives). Simulation
// and M10K configuration power up to zero, as MAME's allocation does.
//
// M10: the array is true dual-port (nb1_dp_ram, two 32K x 8 lanes, the same
// 64 M10K as the M5 single-port store). Port A is the unchanged CPU port;
// port B is the renderer's read-only tile-code port (nb1_c123_render), one
// clk_sys of read latency, always enabled. A CPU write and a video read of
// the same word in the same clk_sys return old or new data for that one read
// (M10K mixed-port behaviour); the renderer reads every tile once per line,
// so at worst one tile on one line shows the previous code.
module nb1_c123_vram (
    input  wire        clk_sys,
    input  wire        access,
    input  wire        write,
    input  wire [15:1] addr,       // byte offset $0000-$FFFF, bit 0 implicit 0
    input  wire [15:0] wdata,
    input  wire [1:0]  be,         // [1] even/MSB byte, [0] odd/LSB byte
    output wire [15:0] rdata,

    // M10 video read port (word address)
    input  wire [14:0] vid_addr,
    output wire [15:0] vid_data
);
    nb1_dp_ram #(.WORDS(32768), .AW(15)) vram (
        .clk_sys(clk_sys),
        .en_a(access), .we_a(write), .addr_a(addr), .wdata_a(wdata), .be_a(be), .rdata_a(rdata),
        .en_b(1'b1), .we_b(1'b0), .addr_b(vid_addr), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(vid_data));
endmodule
