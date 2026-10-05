// Namco NB-2 MiSTer core -- 68EC020 address map classes.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The NB-1 core's nb1_cpu_map_pkg (docs/PROVENANCE.md) for MAME 0.289's namconb2_state::maincpu_am
// [MAME-SOURCE namconb1.cpp 1072-1094] (docs/NB2_HARDWARE_REFERENCE.md section 2). Nothing here names a game.
// Unmapped addresses read 0 and ignore writes, as in MAME (measured: $300000, $56C000, $B00000 read $0000).
package nb2_cpu_map_pkg;

    typedef enum logic [3:0] {
        CLS_ROM     = 4'd0,   // $000000-$0FFFFF program ROM; $400000-$4FFFFF data ROM (both through the ROM cache)
        CLS_WRAM    = 4'd1,   // $208000-$2FFFFF work RAM; $1C0000-$1CFFFF RAM (block RAM pages or SDRAM)
        CLS_SHARE   = 4'd2,   // $200000-$207FFF shared RAM (C75 $4000-$BFFF)
        CLS_RNG     = 4'd3,   // $1E4000-$1E4003 random generator
        CLS_CPUREG  = 4'd4,   // $F00000-$F0001F IRQ levels/acks, watchdog, C75 control (reads $FF)
        CLS_EEPROM  = 4'd5,   // $A00000-$A007FF 2816
        CLS_OBJRAM  = 4'd6,   // $600000-$61FFFF C355 sprite RAM
        CLS_OBJPOS  = 4'd7,   // $620000-$620007 C355 position
        CLS_REGS    = 4'd8,   // $640000-$64000F RAM, $900008-$90000F sprite bank, $940000-$94000F tile bank,
                              // $980000-$98000F ROZ bank
        CLS_VRAM    = 4'd9,   // $680000-$68FFFF C123 tile VRAM
        CLS_TMCTL   = 4'd10,  // $6C0000-$6C003F C123 control
        CLS_ROZRAM  = 4'd11,  // $700000-$71FFFF C169 ROZ VRAM
        CLS_ROZCTL  = 4'd12,  // $740000-$74001F C169 control
        CLS_C116    = 4'd13,  // $800000-$807FFF C116 palette + registers
        CLS_KEYCUS  = 4'd14,  // $C00000-$C0001F KEYCUS (writes ignored)
        CLS_UNMAP   = 4'd15
    } cls_t;

    function automatic cls_t classify(input logic [23:0] a);
        if (a < 24'h100000)                              return CLS_ROM;
        if (a >= 24'h400000 && a < 24'h500000)           return CLS_ROM;
        if (a >= 24'h1C0000 && a < 24'h1D0000)           return CLS_WRAM;
        if (a >= 24'h1E4000 && a < 24'h1E4004)           return CLS_RNG;
        if (a >= 24'h200000 && a < 24'h208000)           return CLS_SHARE;
        if (a >= 24'h208000 && a < 24'h300000)           return CLS_WRAM;
        if (a >= 24'h600000 && a < 24'h620000)           return CLS_OBJRAM;
        if (a >= 24'h620000 && a < 24'h620008)           return CLS_OBJPOS;
        if (a >= 24'h640000 && a < 24'h640010)           return CLS_REGS;
        if (a >= 24'h680000 && a < 24'h690000)           return CLS_VRAM;
        if (a >= 24'h6C0000 && a < 24'h6C0040)           return CLS_TMCTL;
        if (a >= 24'h700000 && a < 24'h720000)           return CLS_ROZRAM;
        if (a >= 24'h740000 && a < 24'h740020)           return CLS_ROZCTL;
        if (a >= 24'h800000 && a < 24'h808000)           return CLS_C116;
        if (a >= 24'h900008 && a < 24'h900010)           return CLS_REGS;
        if (a >= 24'h940000 && a < 24'h940010)           return CLS_REGS;
        if (a >= 24'h980000 && a < 24'h980010)           return CLS_REGS;
        if (a >= 24'hA00000 && a < 24'hA00800)           return CLS_EEPROM;
        if (a >= 24'hC00000 && a < 24'hC00020)           return CLS_KEYCUS;
        if (a >= 24'hF00000 && a < 24'hF00020)           return CLS_CPUREG;
        return CLS_UNMAP;
    endfunction

    // Work-RAM page of a CLS_WRAM address: 0..56 = the 4 KiB pages $208000-$240FFF (hot-page candidates),
    // 63 = elsewhere (always SDRAM).
    function automatic logic [5:0] wram_page(input logic [23:0] a);
        logic [23:0] d;
        d = a - 24'h208000;
        if (a >= 24'h208000 && a < 24'h241000) return d[17:12];
        return 6'd63;
    endfunction

    // SDRAM offset of a CLS_WRAM address inside region WRAM (512 KiB): $208000-$27FFFF -> $08000-$7FFFF;
    // $1C0000-$1C7FFF -> $00000-$07FFF ($1C8000-$1CFFFF mirror it; neither game touches $1C0000-$1CFFFF and
    // $280000-$2FFFFF mirrors $200000-$27FFFF the same way: both games write $260000 / $27C000 once at reset and
    // never read them, docs/NB2_HARDWARE_REFERENCE.md 2.1).
    function automatic logic [18:0] wram_offset(input logic [23:0] a);
        if (a[23:20] == 4'h1) return {4'd0, a[14:0]};
        return a[18:0];
    endfunction

endpackage
