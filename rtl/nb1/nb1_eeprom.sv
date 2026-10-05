// Namco NB-1 MiSTer core -- CPU-visible parallel EEPROM (M6) + persistence port (M18).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// MAME maps NB-1 $580000-$5807FF to an EEPROM_2816 (eeprom_parallel_28xx,
// 2048 x 8) with 8-bit read/write handlers and no unit mask
// [MAME-SOURCE namconb1.cpp maincpu_am, machine/eeprompar.cpp]. MAME
// therefore presents the device on every byte lane with byte offset
// A10:A0: the 68EC020 sees 2 KiB of contiguous big-endian bytes, and word
// and longword accesses read/write consecutive EEPROM cells
// [MAME-CONFIRMED, docs/M6_RESEARCH.md section 3]. Nebulas Ray relies on
// this: it validates the header with cmpm.l and copies settings with
// move.l/move.w reads. There are no mirrors inside or outside the window.
//
// On the 16-bit TG68K cycle this is exactly a 1K x 16 byte-enabled RAM:
// word index A10:A1, D15:8 = even byte, D7:0 = odd byte.
//
// Write timing: MAME builds eeprompar with EMULATE_POLLING 0, so a write is
// visible to the next read (no busy period, no DQ7 data-polling
// complement). Every Nebulas Ray EEPROM store is followed by a whole-byte
// read-back poll that exits on equality, so an immediate write exits that
// poll on its first iteration with the same program flow as MAME
// (docs/M6_RESEARCH.md section 5). A real 28C16's millisecond write cycle
// is not modelled.
//
// Power-on and reset: MAME's nvram_default fills a set without an "eeprom"
// region (nebulray has none) with $FF, the erased state. The array stores
// the complement of each byte, so the M10K configuration/simulation zero
// power-up reads as $FF without an initialisation file. Reset does not
// touch the array (MAME's reset only clears the write-completion timer; the
// part is non-volatile).
//
// M18 persistence port (docs/M18_IMPLEMENTATION.md): the same 2 KiB array
// has a second, independent port for nb1_nvram (MiSTer NVRAM load/save).
// The storage is nb1_dp_ram, the true-dual-port pattern proven by the M8
// shared RAM: two 1K x 8 TDP arrays, the same 2 M10K as the M6 single-port
// 1K x 16. One write port per RAM port -- no multi-write-port structure, no
// copy of the array, no arbitration on the CPU path: port A = 68EC020
// (unchanged timing), port B = persistence only. Port B uses the CPU word
// view (D15:8 = even byte) and plain data; the complement is applied here
// for both ports. A port-B write never raises `cpu_write`, so restoring a
// saved image cannot mark the EEPROM dirty.
module nb1_eeprom (
    input  wire        clk_sys,
    input  wire        access,
    input  wire        write,
    input  wire [10:1] addr,       // byte offset $000-$7FF, bit 0 implicit 0
    input  wire [15:0] wdata,
    input  wire [1:0]  be,         // [1] even/MSB byte, [0] odd/LSB byte
    output wire [15:0] rdata,
    output wire        cpu_write,  // M18: one pulse per committed CPU write cycle
    // M18 persistence port (nb1_nvram)
    input  wire        nv_en,
    input  wire        nv_we,
    input  wire [10:1] nv_addr,
    input  wire [15:0] nv_wdata,
    output wire [15:0] nv_rdata
);
    wire [15:0] q_n, nv_q_n;
    nb1_dp_ram #(.WORDS(1024), .AW(10)) cells (
        .clk_sys(clk_sys),
        .en_a(access), .we_a(write), .addr_a(addr), .wdata_a(~wdata), .be_a(be), .rdata_a(q_n),
        .en_b(nv_en), .we_b(nv_we), .addr_b(nv_addr), .wdata_b(~nv_wdata), .be_b(2'b11), .rdata_b(nv_q_n));
    assign rdata     = ~q_n;
    assign nv_rdata  = ~nv_q_n;
    assign cpu_write = access && write && (be != 2'b00);
endmodule
