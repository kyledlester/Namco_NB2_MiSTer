// Namco NB-1 MiSTer core -- C116 palette storage (M4 CPU port, M10 video port).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// MAME's namco_c116 device and the documented PCB wiring expose a 32 KiB,
// byte-addressed CPU window containing three 8 KiB x 8 palette SRAMs and
// eight 16-bit C116 registers:
//
//   A12:A11 = 00 red, 01 green, 10 blue, 11 registers
//   palette index = {A14:A13, A10:A0} (8192 entries/component)
//   registers use A3:A1; A14:A13 and A10:A4 are ignored
//
// The 68EC020/TG68K adapter presents a 16-bit big-endian cycle. Each color
// plane is therefore stored as 4096 16-bit words: the even pen byte is
// D15:8 and the odd pen byte is D7:0.
//
// M4 stores C116 register bytes and implements their mirrors. Palette data
// is not cleared by reset (matching external SRAM); simulation/M10K power-up
// is zero and Nebulas Ray clears/tests the storage before use.
//
// M10 video side [MAME-SOURCE namco_c116.cpp, namconb1.cpp screen_update]:
//  * each plane is true dual-port (nb1_dp_ram, same 8 M10K per plane as the
//    M4 single-port store); port A is the unchanged CPU port, port B the
//    read-only pen port. vid_pen (13-bit pen = MAME colour index) is
//    registered by the RAM; vid_rgb is valid one clk_sys later and holds.
//    The MAME pen -> RAM mapping is the CPU one: pen = {A14:A13, A10:A0}.
//  * registers 0-3 (clip left/right/top/bottom) are exported for the C116
//    output window (nb1_video_out). Registers 4/5 (raster IRQ H/line) are
//    exported for the later POS interrupt; nothing uses them in M10.
//  * A CPU write and a video read of the same pen in the same clk_sys give
//    that one pixel old or new data (M10K mixed-port behaviour).
module nb1_c116_palette (
    input  wire        clk_sys,
    input  wire        reset,
    input  wire        access,
    input  wire        write,
    input  wire [14:1] addr,       // byte offset $0000-$7fff, bit 0 implicit 0
    input  wire [15:0] wdata,
    input  wire [1:0]  be,         // [1] even/MSB byte, [0] odd/LSB byte
    output reg  [15:0] rdata = 16'h0000,

    // M10 video port
    input  wire [12:0] vid_pen,
    output wire [23:0] vid_rgb,    // {R, G, B}, one clk_sys after vid_pen
    output wire [127:0] regs_out   // {reg7, ..., reg0}, 16 bits each
);
    wire [1:0]  select = addr[12:11];
    wire [11:0] word_addr = {addr[14:13], addr[10:1]};
    wire [2:0]  reg_addr = addr[3:1];
    wire [15:0] q_r, q_g, q_b;
    wire [15:0] v_r, v_g, v_b;
    wire [11:0] vid_word = {vid_pen[12:11], vid_pen[10:1]};

    nb1_dp_ram #(.WORDS(4096), .AW(12)) ram_r (
        .clk_sys(clk_sys), .en_a(access && select == 2'b00), .we_a(write), .addr_a(word_addr),
        .wdata_a(wdata), .be_a(be), .rdata_a(q_r),
        .en_b(1'b1), .we_b(1'b0), .addr_b(vid_word), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(v_r));

    nb1_dp_ram #(.WORDS(4096), .AW(12)) ram_g (
        .clk_sys(clk_sys), .en_a(access && select == 2'b01), .we_a(write), .addr_a(word_addr),
        .wdata_a(wdata), .be_a(be), .rdata_a(q_g),
        .en_b(1'b1), .we_b(1'b0), .addr_b(vid_word), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(v_g));

    nb1_dp_ram #(.WORDS(4096), .AW(12)) ram_b (
        .clk_sys(clk_sys), .en_a(access && select == 2'b10), .we_a(write), .addr_a(word_addr),
        .wdata_a(wdata), .be_a(be), .rdata_a(q_b),
        .en_b(1'b1), .we_b(1'b0), .addr_b(vid_word), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(v_b));

    // even pen = D15:8 of the stored word
    reg vid_odd = 1'b0;
    always @(posedge clk_sys) vid_odd <= vid_pen[0];
    assign vid_rgb = vid_odd ? {v_r[7:0], v_g[7:0], v_b[7:0]} : {v_r[15:8], v_g[15:8], v_b[15:8]};

    reg [15:0] regs [0:7];
    reg [15:0] q_regs = 16'h0000;
    integer i;
    always @(posedge clk_sys) begin
        if (reset) begin
            for (i = 0; i < 8; i = i + 1) regs[i] <= 16'h0000;
            q_regs <= 16'h0000;
        end else if (access && select == 2'b11) begin
            if (write) begin
                if (be[1]) regs[reg_addr][15:8] <= wdata[15:8];
                if (be[0]) regs[reg_addr][7:0]  <= wdata[7:0];
            end
            q_regs <= regs[reg_addr];
        end
    end
    genvar gr;
    generate for (gr = 0; gr < 8; gr++) begin : g_regs
        assign regs_out[gr*16 +: 16] = regs[gr];
    end endgenerate

    always_comb begin
        case (select)
            2'b00: rdata = q_r;
            2'b01: rdata = q_g;
            2'b10: rdata = q_b;
            default: rdata = q_regs;
        endcase
    end
endmodule
