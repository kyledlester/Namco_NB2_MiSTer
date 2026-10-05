// Namco NB-1 MiSTer core -- logical ROM/memory map (M2).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// One table defines every logical storage region of the NB-1 platform:
// its id, its window size and where it lives in SDRAM. Nothing here names a
// game. Clients address memory as (region, byte offset); only this package
// knows physical placement, so regions can move without touching clients.
//
// Byte order [IMPLEMENTATION-DECISION, docs/M2_RESEARCH.md section 7]:
// every region holds MAME's region bytes in MAME's byte order, and a 32-bit
// read at offset A returns {byte A, A+1, A+2, A+3} (big-endian lanes, the
// 68EC020 bus order: D31-24 = A+0). C75 (16-bit little-endian) and byte
// clients select lanes themselves; no region-specific swapping exists.
//
// MRA ioctl index-0 stream offset == SDRAM byte address (identity). The MRA
// concatenates the regions in physical order, each padded to its window.
//
//   id  region    MAME region        window      SDRAM base
//   3   OBJ       c355spr            16 MiB      0x0000000
//   4   CHR       c123tmap            4 MiB      0x1000000
//   2   VOICE     c352 (populated)    2 MiB      0x1400000
//   0   PROG      maincpu             1 MiB      0x1600000
//   5   SHAPE     c123tmap:mask     512 KiB      0x1700000
//   1   C75DATA   c75data           512 KiB      0x1780000
//   6   C75BIOS   namcoc75:internal  16 KiB      0x1800000
//   7   KEYDATA   proms (keycus)     32 B        0x1804000
//   stream / populated end                        0x1804020
//   0x1804020-0x1FFFFFF is unassigned (reserved; see M2_RESEARCH).
//
// Written with case statements and packed-struct returns only, so Quartus
// 17.0 Lite and ModelSim ASE 10.5b accept it unchanged.
package nb1_mem_pkg;

    localparam int NREGIONS = 8;

    localparam logic [3:0] REG_PROG    = 4'd0;
    localparam logic [3:0] REG_C75DATA = 4'd1;
    localparam logic [3:0] REG_VOICE   = 4'd2;
    localparam logic [3:0] REG_OBJ     = 4'd3;
    localparam logic [3:0] REG_CHR     = 4'd4;
    localparam logic [3:0] REG_SHAPE   = 4'd5;
    localparam logic [3:0] REG_C75BIOS = 4'd6;
    localparam logic [3:0] REG_KEYDATA = 4'd7;

    localparam logic [24:0] STREAM_END = 25'h1804020;   // bytes in the index-0 stream

    // SDRAM byte base of a region (25 bits = 32 MiB).
    function automatic logic [24:0] region_base(input logic [3:0] region);
        case (region)
            REG_PROG:    region_base = 25'h1600000;
            REG_C75DATA: region_base = 25'h1780000;
            REG_VOICE:   region_base = 25'h1400000;
            REG_OBJ:     region_base = 25'h0000000;
            REG_CHR:     region_base = 25'h1000000;
            REG_SHAPE:   region_base = 25'h1700000;
            REG_C75BIOS: region_base = 25'h1800000;
            REG_KEYDATA: region_base = 25'h1804000;
            default:     region_base = 25'h0000000;
        endcase
    endfunction

    // Window size in bytes. 0 = no such region.
    function automatic logic [25:0] region_size(input logic [3:0] region);
        case (region)
            REG_PROG:    region_size = 26'h0100000;   // 1 MiB
            REG_C75DATA: region_size = 26'h0080000;   // 512 KiB
            REG_VOICE:   region_size = 26'h0200000;   // 2 MiB
            REG_OBJ:     region_size = 26'h1000000;   // 16 MiB
            REG_CHR:     region_size = 26'h0400000;   // 4 MiB
            REG_SHAPE:   region_size = 26'h0080000;   // 512 KiB
            REG_C75BIOS: region_size = 26'h0004000;   // 16 KiB
            REG_KEYDATA: region_size = 26'h0000020;   // 32 B
            default:     region_size = 26'h0000000;
        endcase
    endfunction

    typedef struct packed {
        logic        oob;    // unknown region or offset outside the window
        logic [24:0] phys;   // SDRAM byte address; meaningless when oob
    } phys_t;

    function automatic phys_t region_to_phys(input logic [3:0] region,
                                             input logic [24:0] offset);
        phys_t r;
        r.oob  = ({1'b0, offset} >= region_size(region));
        r.phys = region_base(region) + offset;
        return r;
    endfunction

    typedef struct packed {
        logic        hit;     // 0: address lies outside every region
        logic [3:0]  region;
        logic [24:0] offset;
    } loc_t;

    // Stream (== SDRAM) byte address -> (region, offset).
    function automatic loc_t stream_to_region(input logic [26:0] addr);
        loc_t r;
        r.hit = 1'b1;
        if      (addr < 27'h1000000) r.region = REG_OBJ;
        else if (addr < 27'h1400000) r.region = REG_CHR;
        else if (addr < 27'h1600000) r.region = REG_VOICE;
        else if (addr < 27'h1700000) r.region = REG_PROG;
        else if (addr < 27'h1780000) r.region = REG_SHAPE;
        else if (addr < 27'h1800000) r.region = REG_C75DATA;
        else if (addr < 27'h1804000) r.region = REG_C75BIOS;
        else if (addr < 27'h1804020) r.region = REG_KEYDATA;
        else begin
            r.region = 4'hF;
            r.hit    = 1'b0;
        end
        r.offset = addr[24:0] - region_base(r.region);
        return r;
    endfunction

endpackage
