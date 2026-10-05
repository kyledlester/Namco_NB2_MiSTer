// Namco NB-1 MiSTer core -- generic C3xx KEYCUS (M7).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// MAME maps NB-1 $6E0000-$6E001F to namconb1_state::custom_key_r, a 32-bit
// read handler, and ignores writes (nopw) [MAME-SOURCE namconb1.cpp
// maincpu_am, custom_key_r]. Per game it returns a constant "ID" in one
// 16-bit half of one longword and a changing value ("never the same twice
// in a row") in another half; every other word reads $0000. On the 16-bit
// TG68K bus that is 16 words at word index A4:A1: word i is byte address
// $6E0000 + 2i, D15:8 its even byte. The ID is the part number in binary
// (C366 = $016E) and which words answer is a property of the fitted part,
// so both come from the MRA's board record (nb1_board_config); nothing here
// names a game. See docs/M7_RESEARCH.md.
//
// Mode 0 (no record, invalid record, or a board without a KEYCUS answer):
// every word reads $0000, as MAME's default path. Mode 1: ID word -> ID,
// changing word -> the generator below, any other word -> $0000. If the two
// indices are equal the ID wins.
//
// Changing value [IMPLEMENTATION-DECISION, docs/M7_RESEARCH.md section 5]:
// MAME uses host rand(); the physical C3xx generator is [UNKNOWN]. This is
// a 16-bit maximal-length Galois LFSR (taps $B400, period 65,535), stepped
// once per KEYCUS read cycle of ANY word (MAME re-rolls on every
// custom_key_r call), and a read of the changing word returns the state
// before its step. Any two -- in fact any 65,535 -- consecutive values
// differ, which is what Nebulas Ray's boot check ($021A1E-$021A2E: three
// reads, the first must differ from the other two) and every other known
// use require. The sequence depends only on the number of KEYCUS reads, not
// on time, so a run is deterministic and the MAME reference script can
// reproduce it exactly (scripts/mame/nb1_m4_ref.lua policy m7). CPU reset
// reloads the seed, so every boot sees the same sequence.
//
// Bus: synchronous like the CPU RAMs. A read registers its word in the
// access cycle and rdata holds it until the next read; the bus completes the
// cycle one clk_sys later. Writes complete immediately and change nothing
// (the $6E0002 "kick" writes are counted by nb1_main_bus only).
module nb1_keycus #(
    parameter [15:0] SEED = 16'hACE1   // any nonzero value; $ACE1 as in the NA-1 core
) (
    input  wire        clk_sys,
    input  wire        reset,         // CPU reset: reload the generator seed
    input  wire        access,        // bc_start of a KEYCUS cycle
    input  wire        write,
    input  wire [4:1]  addr,          // word index 0-15

    input  wire        cfg_valid,
    input  wire [7:0]  cfg_mode,
    input  wire [3:0]  cfg_id_word,
    input  wire [15:0] cfg_id,
    input  wire [3:0]  cfg_rnd_word,

    output reg  [15:0] rdata = 16'd0,
    output reg  [15:0] lfsr  = SEED     // exposed for benches/diagnostics
);
    wire        fitted    = cfg_valid && (cfg_mode == 8'd1);
    wire [15:0] lfsr_next = {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0000);

    always @(posedge clk_sys) begin
        if (reset) begin
            lfsr  <= SEED;
            rdata <= 16'd0;
        end else if (access && !write) begin
            rdata <= !fitted                  ? 16'd0 :
                     (addr == cfg_id_word)    ? cfg_id :
                     (addr == cfg_rnd_word)   ? lfsr : 16'd0;
            lfsr  <= lfsr_next;
        end
    end
endmodule
