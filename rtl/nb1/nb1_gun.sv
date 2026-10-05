// Namco NB-1 MiSTer core -- light-gun I/O board (Point Blank / Gun Bullet) (M25).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// [MAME-SOURCE namconb1.cpp gunbulet_state] The gun games add an I/O board read at
// $100000-$10001F (32-bit handler gun_r, the value in bits 31:24 of each longword, i.e. the
// byte at $100000 + 4n; every other byte reads 0):
//   n = 0, 1  Y of player 2      0x0F + LIGHT1_Y * 224 / 255
//   n = 2, 3  X of player 2      0x26 + LIGHT1_X * 288 / 314   (8 bits: wraps past $FF)
//   n = 4, 5  Y of player 1      0x0F + LIGHT0_Y * 224 / 255
//   n = 6, 7  X of player 1      0x26 + LIGHT0_X * 288 / 314
// LIGHTn_X/Y are MAME light-gun ports, 0..255 across the visible screen (288 x 224). The trigger is
// the player's button 1 on the normal C75 input port (nb1_inputs); MAME has no other gun signal.
// The game turns these raw counts into screen positions with the calibration in its EEPROM
// (MAME's default "eeprom" region, loaded by the MRA), so matching MAME's mapping keeps the
// default calibration right.
//
// [IMPLEMENTATION] Aim source:
//   src 0  MiSTer joysticks: player n's left analog stick (hps_io joystick_l_analog_n, signed
//          -128..127 across the screen). Main_MiSTer delivers a GunCon 2 (CRT, calibrated per core
//          with F10 in the OSD), Sinden/Gun4IR/Wiimote guns and plain analog sticks this way, so the
//          port value is simply analog + 128.
//   src 1  Mouse for player 1 (relative motion accumulated, clamped to 0..255); player 2 stays on
//          joystick 2.
// Exact integer forms of MAME's arithmetic for every 0..255 port value (checked exhaustively):
//   v * 288 / 314 = (v * 3757) >> 12,  v * 224 / 255 = (v * 28785) >> 15.
// Crosshair: a small cross per player at the aimed pixel, drawn in game coordinates
// (x = port * 288 / 256, y = port * 224 / 256), player 1 red-white, player 2 blue-white.
module nb1_gun (
    input  wire        clk_sys,
    input  wire        enable,            // board record: light-gun I/O board fitted
    input  wire        src_mouse,         // OSD: 1 = player 1 aims with the mouse
    input  wire [15:0] joy_a0,            // hps_io joystick_l_analog_0 {Y, X}, signed
    input  wire [15:0] joy_a1,
    input  wire [24:0] ps2_mouse,         // hps_io ps2_mouse

    // the four gun counts for the 68020 read port (nb1_main_bus decodes $100000-$10001F):
    // {X p1, Y p1, X p2, Y p2}; all zero when no gun board is fitted
    output wire [31:0] counts,

    // crosshair
    input  wire        cross_on,
    input  wire [8:0]  hcount,
    input  wire [8:0]  vcount,
    output reg         cross_hit = 1'b0,  // a crosshair pixel at (hcount, vcount)
    output reg  [23:0] cross_rgb = 24'h000000
);
    // ---------------------------------------------------------------- port values 0..255
    reg  [7:0] px0 = 8'h80, py0 = 8'h80, px1 = 8'h80, py1 = 8'h80;
    reg  [7:0] mx = 8'h80, my = 8'h80;
    reg        ms_q = 1'b0;
    wire signed [9:0] mx_n = $signed({2'b00, mx}) + $signed({{2{ps2_mouse[4]}}, ps2_mouse[15:8]});
    wire signed [9:0] my_n = $signed({2'b00, my}) - $signed({{2{ps2_mouse[5]}}, ps2_mouse[23:16]});
    always @(posedge clk_sys) begin
        ms_q <= ps2_mouse[24];
        if (ms_q != ps2_mouse[24]) begin          // one mouse report
            mx <= (mx_n < 0) ? 8'd0 : (mx_n > 255) ? 8'd255 : mx_n[7:0];
            my <= (my_n < 0) ? 8'd0 : (my_n > 255) ? 8'd255 : my_n[7:0];
        end
        px0 <= src_mouse ? mx : {~joy_a0[7],  joy_a0[6:0]};    // signed + 128
        py0 <= src_mouse ? my : {~joy_a0[15], joy_a0[14:8]};
        px1 <= {~joy_a1[7],  joy_a1[6:0]};
        py1 <= {~joy_a1[15], joy_a1[14:8]};
    end

    // ---------------------------------------------------------------- gun counts (MAME gun_r)
    function automatic [7:0] cnt_x(input [7:0] v);
        reg [19:0] p;
        begin p = v * 20'd3757;  cnt_x = 8'h26 + p[19:12]; end
    endfunction
    function automatic [7:0] cnt_y(input [7:0] v);
        reg [22:0] p;
        begin p = v * 23'd28785; cnt_y = 8'h0F + p[22:15]; end
    endfunction
    reg [7:0] gy1 = 8'h0F, gx1 = 8'h26, gy0 = 8'h0F, gx0 = 8'h26;
    always @(posedge clk_sys) begin
        gy1 <= cnt_y(py1); gx1 <= cnt_x(px1);
        gy0 <= cnt_y(py0); gx0 <= cnt_x(px0);
    end
    assign counts = enable ? {gx0, gy0, gx1, gy1} : 32'h0;

    // ---------------------------------------------------------------- crosshair (game pixels)
    // x = port * 288 / 256 = port + port / 8, y = port * 224 / 256 = port - port / 8
    reg [8:0] cx0, cy0, cx1, cy1;
    always @(posedge clk_sys) begin
        cx0 <= {1'b0, px0} + px0[7:3];  cy0 <= {1'b0, py0} - py0[7:3];
        cx1 <= {1'b0, px1} + px1[7:3];  cy1 <= {1'b0, py1} - py1[7:3];
    end
    function automatic [1:0] xhair(input [8:0] cx, input [8:0] cy, input [8:0] h, input [8:0] v);
        reg [9:0] dx, dy;
        reg arm, rim;
        begin
            dx  = (h >= cx) ? h - cx : cx - h;
            dy  = (v >= cy) ? v - cy : cy - v;
            arm = ((dx == 0) && (dy <= 6) && (dy >= 2)) || ((dy == 0) && (dx <= 6) && (dx >= 2));
            rim = ((dx <= 1) && (dy <= 7) && (dy >= 1)) || ((dy <= 1) && (dx <= 7) && (dx >= 1));
            xhair = {arm, rim};   // arm = colour, rim = white outline
        end
    endfunction
    wire [1:0] c0 = xhair(cx0, cy0, hcount, vcount);
    wire [1:0] c1 = xhair(cx1, cy1, hcount, vcount);
    always @(posedge clk_sys) begin
        cross_hit <= enable && cross_on && (|c0 || |c1);
        cross_rgb <= c0[1] ? 24'hFF2020 : c1[1] ? 24'h2060FF : 24'hFFFFFF;
    end
endmodule
