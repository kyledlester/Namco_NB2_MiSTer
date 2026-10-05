// Namco NB-1 MiSTer core -- 68EC020 address map classes (M3).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// One function classifies a 24-bit 68EC020 byte address (the EC020 has 24
// address lines, so A31-A24 never reach the board) into the NB-1 map of
// MAME's namconb1_state::maincpu_am [MAME-SOURCE namconb1.cpp line 1048].
// Nothing here names a game. See docs/M3_RESEARCH.md section 3.
//
// M3 implements CLS_ROM and the three CPU RAMs; M4 adds CPU-visible C116
// palette/register storage; M5 adds CPU-visible C123 tile VRAM (CLS_VRAM
// only; C123 control CLS_TMCTL stayed unimplemented until M10, docs/M5_RESEARCH.md);
// M6 adds the CPU-visible 2816 EEPROM (CLS_EEPROM, docs/M6_RESEARCH.md);
// M7 adds the generic C3xx KEYCUS (CLS_KEYCUS, docs/M7_RESEARCH.md);
// M10 adds the C123 control registers (CLS_TMCTL, docs/M10_RESEARCH.md);
// M11 adds the C355 sprite RAM, position and bank (docs/M11_RESEARCH.md).
// Every other class is "unimplemented": reads
// return all ones, writes are dropped, and both are counted and latched for
// debugging by nb1_main_bus. CLS_RAMX marks
// addresses MAME maps as RAM but M3 does not back with storage (see the RAM
// sizing note in docs/M3_IMPLEMENTATION.md); it is kept separate from the
// device classes so an access there is visibly a sizing question, not a
// missing device.
package nb1_cpu_map_pkg;

    // Bytes of the $1C0000 RAM that M3 implements. MAME maps 64 KiB; Nebulas
    // Ray was measured to use $1C0000-$1C03FF only (M0 capture, 1800 frames:
    // stack top $1C0400, vectors, RAM ISR). [IMPLEMENTATION-DECISION]
    // M25: 4 KiB (was 16 KiB). Every NB-1 game measured (MAME captures, docs/M25_IMPLEMENTATION.md) stays
    // below $1C0500 (Point Blank's RAM test is the largest: $1C0000-$1C04FF). The same block RAM also backs
    // the 4 KiB window RAM24 at $240000: J-League Soccer V-Shoot keeps a stack just below $240400 and Great
    // Sluggers runs a RAM test over $240000-$2403FF; no game touches the rest of $240000-$2FFFFF except
    // single writes at $260000 / $27C000 that are never read back.
    localparam int RAM1C_BYTES = 4096;
    localparam int RAM24_BYTES = 4096;

    typedef enum logic [3:0] {
        CLS_ROM     = 4'd0,   // $000000-$0FFFFF program ROM (SDRAM region PROG)
        CLS_RAM1C   = 4'd1,   // $1C0000-$1C0FFF stack/vector RAM + M25 $240000-$240FFF (one BRAM)
        CLS_SHARE   = 4'd2,   // $200000-$207FFF shared RAM (BRAM; C75 port in M6)
        CLS_WRAM    = 4'd3,   // $208000-$23FFFF work RAM (BRAM)
        CLS_RAMX    = 4'd4,   // MAME RAM not backed in M3: $1C4000-$1CFFFF, $240000-$2FFFFF
        CLS_RNG     = 4'd5,   // $1E4000-$1E4003 random generator; M25: also the gun I/O $100000-$10001F
        CLS_CPUREG  = 4'd6,   // $400000-$40001F IRQ/C75/watchdog registers
        CLS_EEPROM  = 4'd7,   // $580000-$5807FF EEPROM 2816 (M6)
        CLS_OBJRAM  = 4'd8,   // $600000-$61FFFF C355 sprite RAM
        CLS_OBJPOS  = 4'd9,   // $620000-$620007 C355 position
        CLS_VRAM    = 4'd10,  // $640000-$64FFFF C123 tilemap VRAM (M5)
        CLS_TMCTL   = 4'd11,  // $660000-$66003F C123 control (M10)
        CLS_SPRBANK = 4'd12,  // $680000-$68000F sprite tile bank
        CLS_KEYCUS  = 4'd13,  // $6E0000-$6E001F KEYCUS C3xx (M7)
        CLS_C116    = 4'd14,  // $700000-$707FFF C116 palette/register storage (M4)
        CLS_UNMAP   = 4'd15   // not decoded by MAME at all
    } cls_t;

    function automatic cls_t classify(input logic [23:0] a);
        if (a < 24'h100000)                              return CLS_ROM;
        if (a >= 24'h1C0000 && a < 24'h1C0000 + RAM1C_BYTES) return CLS_RAM1C;
        if (a >= 24'h1C0000 && a < 24'h1D0000)           return CLS_RAMX;
        if (a >= 24'h1E4000 && a < 24'h1E4004)           return CLS_RNG;
        if (a >= 24'h100000 && a < 24'h100020)           return CLS_RNG;   // M25: gun I/O (reads 0 without a gun board)
        if (a >= 24'h200000 && a < 24'h208000)           return CLS_SHARE;
        if (a >= 24'h208000 && a < 24'h240000)           return CLS_WRAM;
        if (a >= 24'h240000 && a < 24'h240000 + RAM24_BYTES) return CLS_RAM1C;   // M25
        if (a >= 24'h240000 && a < 24'h300000)           return CLS_RAMX;
        if (a >= 24'h400000 && a < 24'h400020)           return CLS_CPUREG;
        if (a >= 24'h580000 && a < 24'h580800)           return CLS_EEPROM;
        if (a >= 24'h600000 && a < 24'h620000)           return CLS_OBJRAM;
        if (a >= 24'h620000 && a < 24'h620008)           return CLS_OBJPOS;
        if (a >= 24'h640000 && a < 24'h650000)           return CLS_VRAM;
        if (a >= 24'h660000 && a < 24'h660040)           return CLS_TMCTL;
        if (a >= 24'h680000 && a < 24'h680010)           return CLS_SPRBANK;
        if (a >= 24'h6E0000 && a < 24'h6E0020)           return CLS_KEYCUS;
        if (a >= 24'h700000 && a < 24'h708000)           return CLS_C116;
        return CLS_UNMAP;
    endfunction

    function automatic logic implemented(input cls_t c);
        return (c == CLS_ROM) || (c == CLS_RAM1C) || (c == CLS_SHARE) ||
               (c == CLS_WRAM) || (c == CLS_C116) || (c == CLS_VRAM) ||
               (c == CLS_EEPROM) || (c == CLS_KEYCUS) || (c == CLS_TMCTL) ||
               (c == CLS_OBJRAM) || (c == CLS_OBJPOS) || (c == CLS_SPRBANK) || (c == CLS_RNG);
    endfunction

endpackage
