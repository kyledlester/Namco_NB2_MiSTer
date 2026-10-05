// Namco NB-2 MiSTer core -- C123 tile VRAM (CPU port + renderer port).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The NB-1 core's nb1_c123_vram (docs/PROVENANCE.md) reduced to the 40 KiB both NB-2 games use: MAME maps
// 64 KiB at $680000-$68FFFF, and the MAME profiles of Outfoxies and Mach Breakers (9,000 frames with autoplay,
// docs/NB2_HARDWARE_REFERENCE.md section 2) touch only $680000-$689FFF. That window holds the four scrolling
// layers (words $0000-$3FFF) and the two fixed layers ($4008-$47E7), i.e. everything the C123 renderer reads.
// Words $5000-$7FFF read $0000 (MAME's power-up contents) and ignore writes. Saves 24 M10K blocks.
module nb2_c123_vram (
    input  wire        clk_sys,
    input  wire        access,
    input  wire        write,
    input  wire [15:1] addr,       // byte offset $0000-$FFFF, bit 0 implicit 0
    input  wire [15:0] wdata,
    input  wire [1:0]  be,         // [1] even/MSB byte, [0] odd/LSB byte
    output wire [15:0] rdata,

    input  wire [14:0] vid_addr,
    output wire [15:0] vid_data
);
    localparam int WORDS = 20480;   // 40 KiB
    wire        in_a = (addr < WORDS);
    reg         in_q = 1'b0;
    wire [15:0] q;
    always @(posedge clk_sys) if (access) in_q <= in_a;
    assign rdata = in_q ? q : 16'h0000;

    nb1_dp_ram #(.WORDS(WORDS), .AW(15)) vram (
        .clk_sys(clk_sys),
        .en_a(access && in_a), .we_a(write), .addr_a(addr), .wdata_a(wdata), .be_a(be), .rdata_a(q),
        .en_b(1'b1), .we_b(1'b0), .addr_b(vid_addr), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(vid_data));
endmodule
