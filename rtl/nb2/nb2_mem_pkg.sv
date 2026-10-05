// Namco NB-2 MiSTer core -- logical ROM/memory map.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// NB-2 counterpart of the NB-1 core's nb1_mem_pkg (same interface: region ids, windows, (region, offset) ->
// physical). One table defines every logical storage region; nothing here names a game.
// docs/ARCHITECTURE.md section 2 has the placement decision.
//
// Physical space (26 bits): 0x0000000-0x1FFFFFF = SDRAM (32 MiB), 0x2000000-0x3FFFFFF = the DDR3 sprite store.
// MRA ioctl index-0 stream offset == physical address (identity), as on NB-1.
//
//   id  region   MAME region        window   base
//   8   ROZ      c169roz            8 MiB    0x0000000
//   4   CHR      c123tmap           8 MiB    0x0800000
//   2   VOICE    c352               8 MiB    0x1000000   (C352 address {A23, A21..A0}; A22 = MAME's reload mirror)
//   5   SHAPE    c123tmap:mask      2 MiB    0x1800000
//   9   ROZMASK  c169roz:mask       2 MiB    0x1A00000
//   0   PROG     maincpu            1 MiB    0x1C00000
//   7   DATA     data               1 MiB    0x1D00000
//   1   C75DATA  c75data          512 KiB    0x1E00000
//   6   C75BIOS  mcu:internal      16 KiB    0x1E80000
//   10  WRAM     (cold work RAM)  512 KiB    0x1F00000   not in the stream
//   3   OBJ      c355spr           20 MiB    0x2000000   DDR3
//   stream end 0x3400000 (0x1E84000-0x1FFFFFF of the stream is zero filler)
//
// Byte order as NB-1: every region holds MAME's region bytes in MAME's order; a 32-bit read at offset A returns
// {byte A, A+1, A+2, A+3} (68EC020 order).
package nb2_mem_pkg;

    localparam logic [3:0] REG_PROG    = 4'd0;
    localparam logic [3:0] REG_C75DATA = 4'd1;
    localparam logic [3:0] REG_VOICE   = 4'd2;
    localparam logic [3:0] REG_OBJ     = 4'd3;
    localparam logic [3:0] REG_CHR     = 4'd4;
    localparam logic [3:0] REG_SHAPE   = 4'd5;
    localparam logic [3:0] REG_C75BIOS = 4'd6;
    localparam logic [3:0] REG_DATA    = 4'd7;
    localparam logic [3:0] REG_ROZ     = 4'd8;
    localparam logic [3:0] REG_ROZMASK = 4'd9;
    localparam logic [3:0] REG_WRAM    = 4'd10;

    localparam logic [26:0] STREAM_END = 27'h3400000;
    localparam logic [25:0] DDR3_BASE  = 26'h2000000;   // physical addresses at/above this are the DDR3 store

    function automatic logic [25:0] region_base(input logic [3:0] region);
        case (region)
            REG_ROZ:     region_base = 26'h0000000;
            REG_CHR:     region_base = 26'h0800000;
            REG_VOICE:   region_base = 26'h1000000;
            REG_SHAPE:   region_base = 26'h1800000;
            REG_ROZMASK: region_base = 26'h1A00000;
            REG_PROG:    region_base = 26'h1C00000;
            REG_DATA:    region_base = 26'h1D00000;
            REG_C75DATA: region_base = 26'h1E00000;
            REG_C75BIOS: region_base = 26'h1E80000;
            REG_WRAM:    region_base = 26'h1F00000;
            REG_OBJ:     region_base = 26'h2000000;
            default:     region_base = 26'h0000000;
        endcase
    endfunction

    function automatic logic [25:0] region_size(input logic [3:0] region);
        case (region)
            REG_ROZ:     region_size = 26'h0800000;
            REG_CHR:     region_size = 26'h0800000;
            REG_VOICE:   region_size = 26'h0800000;
            REG_SHAPE:   region_size = 26'h0200000;
            REG_ROZMASK: region_size = 26'h0200000;
            REG_PROG:    region_size = 26'h0100000;
            REG_DATA:    region_size = 26'h0100000;
            REG_C75DATA: region_size = 26'h0080000;
            REG_C75BIOS: region_size = 26'h0004000;
            REG_WRAM:    region_size = 26'h0080000;
            REG_OBJ:     region_size = 26'h1400000;
            default:     region_size = 26'h0000000;
        endcase
    endfunction

    typedef struct packed {
        logic        oob;    // unknown region or offset outside the window
        logic [25:0] phys;
    } phys_t;

    function automatic phys_t region_to_phys(input logic [3:0] region, input logic [25:0] offset);
        phys_t r;
        r.oob  = (offset >= region_size(region));
        r.phys = region_base(region) + offset;
        return r;
    endfunction

    typedef struct packed {
        logic        hit;
        logic [3:0]  region;
        logic [25:0] offset;
    } loc_t;

    // stream (== physical) byte address -> (region, offset); the filler gap is no region
    function automatic loc_t stream_to_region(input logic [26:0] addr);
        loc_t r;
        r.hit = 1'b1;
        if      (addr < 27'h0800000) r.region = REG_ROZ;
        else if (addr < 27'h1000000) r.region = REG_CHR;
        else if (addr < 27'h1800000) r.region = REG_VOICE;
        else if (addr < 27'h1A00000) r.region = REG_SHAPE;
        else if (addr < 27'h1C00000) r.region = REG_ROZMASK;
        else if (addr < 27'h1D00000) r.region = REG_PROG;
        else if (addr < 27'h1E00000) r.region = REG_DATA;
        else if (addr < 27'h1E80000) r.region = REG_C75DATA;
        else if (addr < 27'h1E84000) r.region = REG_C75BIOS;
        else if (addr >= 27'h2000000 && addr < 27'h3400000) r.region = REG_OBJ;
        else begin
            r.region = 4'hF;
            r.hit    = 1'b0;
        end
        r.offset = addr[25:0] - region_base(r.region);
        return r;
    endfunction

endpackage
