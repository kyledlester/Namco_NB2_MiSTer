// Namco NB-1 MiSTer core -- M1 raster test pattern (not NB-1 hardware).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Deterministic 288x224 pattern in the NB-1 native (unrotated) raster,
// x = hcount 0..287, y = vcount 0..223:
//   * 1-pixel white border on x=0, x=287, y=0, y=223 (all four active edges)
//   * white ticks every 32 px along the top and left border (coordinates)
//   * 16x16 corner blocks: TL red, TR green, BL blue, BR yellow (orientation)
//   * dark-grey grid every 16 px
//   * y 24-55: eight 32-px colour bars left->right, x 16-271:
//     white yellow cyan green magenta red blue grey
//   * y 64-79: horizontal grey ramp, x 16-271, level = x-16 (dark left)
//   * x 276-283, y 24-199: vertical grey ramp (dark top -> bright bottom)
//   * magenta crosshair on the active-area centre (x 143/144, y 111/112)
//   * y 176-183: 16-bit frame counter, 16-px cells, MSB left, lit = 1
//   * y 192-199: 8x8 marker moving 1 px per frame, x 16..263
// Colour is registered once, so it is valid from the second clk_sys cycle of
// each 16-cycle pixel, well before that pixel's ce_pix sample cycle.
module nb1_test_pattern (
    input  wire        clk_sys,
    input  wire        reset,        // restarts the moving marker
    input  wire [8:0]  x,
    input  wire [8:0]  y,
    input  wire        de,
    input  wire        frame_tick,   // one pulse per frame
    input  wire [15:0] frame,
    output reg  [7:0]  r = 8'd0,
    output reg  [7:0]  g = 8'd0,
    output reg  [7:0]  b = 8'd0
);
    localparam [8:0] W = 9'd288, H = 9'd224;

    reg [7:0] marker = 8'd0;           // 0..247 -> x 16..263
    always @(posedge clk_sys)
        if (reset) marker <= 8'd0;
        else if (frame_tick) marker <= (marker == 8'd247) ? 8'd0 : marker + 8'd1;

    wire [8:0] xr  = x - 9'd16;        // 0..255 inside the bar/ramp/counter bands
    wire [8:0] yr  = y - 9'd24;
    wire [8:0] vlv = yr + (yr >> 2);   // 0..218 over y 24..199
    wire [8:0] mx  = {1'b0, marker} + 9'd16;
    wire       band_x = (x >= 9'd16) && (x <= 9'd271);

    reg [23:0] c;
    always @* begin
        c = 24'h000000;
        if (!de)
            c = 24'h000000;
        else if (x == 9'd0 || x == W - 1 || y == 9'd0 || y == H - 1)
            c = 24'hFFFFFF;                                            // border
        else if ((y <= 9'd4 && x[4:0] == 5'd0) || (x <= 9'd4 && y[4:0] == 5'd0))
            c = 24'hFFFFFF;                                            // 32-px ticks
        else if (x >= 9'd2 && x <= 9'd17 && y >= 9'd2 && y <= 9'd17)
            c = 24'hFF0000;                                            // top-left
        else if (x >= W - 18 && x <= W - 3 && y >= 9'd2 && y <= 9'd17)
            c = 24'h00FF00;                                            // top-right
        else if (x >= 9'd2 && x <= 9'd17 && y >= H - 18 && y <= H - 3)
            c = 24'h0000FF;                                            // bottom-left
        else if (x >= W - 18 && x <= W - 3 && y >= H - 18 && y <= H - 3)
            c = 24'hFFFF00;                                            // bottom-right
        else if (band_x && y >= 9'd24 && y <= 9'd55)
            case (xr[7:5])                                             // colour bars
                3'd0: c = 24'hFFFFFF;
                3'd1: c = 24'hFFFF00;
                3'd2: c = 24'h00FFFF;
                3'd3: c = 24'h00FF00;
                3'd4: c = 24'hFF00FF;
                3'd5: c = 24'hFF0000;
                3'd6: c = 24'h0000FF;
                default: c = 24'h808080;
            endcase
        else if (band_x && y >= 9'd64 && y <= 9'd79)
            c = {3{xr[7:0]}};                                          // horizontal ramp
        else if (x >= 9'd276 && x <= 9'd283 && y >= 9'd24 && y <= 9'd199)
            c = {3{vlv[7:0]}};                                         // vertical ramp
        else if ((x >= 9'd143 && x <= 9'd144 && y >= 9'd96 && y <= 9'd127) ||
                 (y >= 9'd111 && y <= 9'd112 && x >= 9'd128 && x <= 9'd159))
            c = 24'hFF00FF;                                            // centre crosshair
        else if (band_x && y >= 9'd176 && y <= 9'd183 && xr[3:0] < 4'd12)
            c = frame[4'd15 - xr[7:4]] ? 24'hFFFFFF : 24'h404040;      // frame counter
        else if (y >= 9'd192 && y <= 9'd199 && x >= mx && x <= mx + 9'd7)
            c = 24'hFFFFFF;                                            // moving marker
        else if (x[3:0] == 4'd0 || y[3:0] == 4'd0)
            c = 24'h303030;                                            // grid
    end

    always @(posedge clk_sys) {r, g, b} <= c;
endmodule
