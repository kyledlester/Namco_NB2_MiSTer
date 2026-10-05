// Namco NB-1 MiSTer core -- M2 ROM/memory status overlay (diagnostic).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Draws a status panel into the unused band y 132..171 of the M1 test
// pattern (native 288x224 raster, unrotated; the panel spans x 16..271).
// Everything outside the panel passes through unchanged.
//
//   bar     y 134..143, full panel width: overall state
//           dark grey = no ROM or no check record, blue = checking,
//           GREEN = ALL PASS, red = a check failed or stream length wrong,
//           magenta = loader error (overrun, write outside map, write error)
//   cells   entry i: x 16+16*(i%16) .. +13; y 146..153 (i<16), 156..163 (i>=16)
//           black = unused, grey = pending, blue = running, green = pass, red = fail
//   flags   y 166..171, x 16..271 in 4 blocks of 32 px (left half):
//           rom_loaded, record_ok, stream length ok, loader clean (green=yes, red=no)
//   runs    y 166..171, right half (x 144..271): check-run counter, 4 bits,
//           MSB left, white = 1. Advances after every completed run (for
//           example after each OSD reset), so a re-check is visible.
// All cell/block selection uses power-of-two geometry (bit slices only).
module nb1_m2_overlay #(parameter int MAX_ENTRIES = 32) (
    input  wire        clk_sys,
    input  wire [8:0]  x,
    input  wire [8:0]  y,
    input  wire        de,
    input  wire [7:0]  in_r, in_g, in_b,

    input  wire        rom_loaded,
    input  wire        loader_error,
    input  wire        record_ok,
    input  wire [5:0]  entry_count,
    input  wire [2*MAX_ENTRIES-1:0] entry_status,
    input  wire        running,
    input  wire        done,
    input  wire        all_pass,
    input  wire        length_ok,
    input  wire [3:0]  run_count,

    output reg  [7:0]  r = 8'd0,
    output reg  [7:0]  g = 8'd0,
    output reg  [7:0]  b = 8'd0
);
    localparam [23:0] BLACK = 24'h000000, DGREY = 24'h404040, GREY = 24'hA0A0A0,
                      BLUE  = 24'h2040FF, GREEN = 24'h00E000, RED  = 24'hFF0000,
                      MAG   = 24'hFF00FF, WHITE = 24'hFFFFFF;

    wire       panel = (y >= 9'd132) && (y <= 9'd171) && (x >= 9'd16) && (x <= 9'd271);
    wire [7:0] px    = x[7:0] - 8'd16;          // 0..255 inside the panel
    wire       row1  = (y >= 9'd156);
    wire [5:0] cell_idx  = {1'b0, row1, px[7:4]};

    reg [1:0]  st;
    always_comb begin
        st = 2'd0;
        for (int i = 0; i < MAX_ENTRIES; i++)
            if (cell_idx == i) st = entry_status[2*i +: 2];
    end

    reg [23:0] c;
    always_comb begin
        c = {in_r, in_g, in_b};
        if (de && panel) begin
            c = BLACK;
            if (y >= 9'd134 && y <= 9'd143) begin
                if (loader_error)                   c = MAG;
                else if (!rom_loaded || !record_ok) c = DGREY;
                else if (running || !done)          c = BLUE;
                else if (all_pass)                  c = GREEN;
                else                                c = RED;
            end else if (((y >= 9'd146 && y <= 9'd153) || (y >= 9'd156 && y <= 9'd163)) &&
                         px[3:0] < 4'd14) begin
                if (cell_idx >= entry_count) c = BLACK;
                else case (st)
                    2'd0:    c = GREY;
                    2'd1:    c = BLUE;
                    2'd2:    c = GREEN;
                    default: c = RED;
                endcase
            end else if (y >= 9'd166 && y <= 9'd171 && px[4:0] < 5'd28) begin
                if (!px[7]) case (px[6:5])
                    2'd0: c = rom_loaded    ? GREEN : RED;
                    2'd1: c = record_ok     ? GREEN : RED;
                    2'd2: c = length_ok     ? GREEN : RED;
                    default: c = !loader_error ? GREEN : RED;
                endcase
                else c = run_count[2'd3 - px[6:5]] ? WHITE : DGREY;
            end
        end
    end

    always @(posedge clk_sys) {r, g, b} <= c;
endmodule
