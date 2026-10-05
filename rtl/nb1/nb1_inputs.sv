// Namco NB-1 MiSTer core -- player controls to the C75 input ports (M14).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The NB-1 has no input register on the 68020 bus: every switch is read by
// the C75 MCU through its ports (nb1_c75: P6 select, P7 = P4/MISC/P1/P2, P3
// on the A-D inputs) and passed to the game by the C75 firmware in a shared
// RAM mailbox (docs/M14_PLAN.md). This module only forms the five MAME
// `namconb1` port bytes, active low [MAME-SOURCE namconb1.cpp INPUT_PORTS
// namconb1]:
//   P1-P4  bit 0 right, 1 left, 2 down, 3 up, 4 button 1, 5 button 2,
//          6 button 3, 7 start
//   MISC   bit 0 Freeze Screen DIP (SW1:2), 1 Service Mode DIP (SW1:1),
//          2 coin 4, 3 coin 3, 4 coin 2, 5 coin 1, 6 Test switch,
//          7 service (service coin)
// from the MiSTer pads (hps_io joystick_n, active high): bit 0 right,
// 1 left, 2 down, 3 up, 4-6 buttons 1-3, 7 start, 8 coin, 9 service
// (CONF_STR "J1,Button 1,Button 2,Button 3,Start,Coin,Service"). Player n's
// coin button is coin n; any player's service button is the service input.
// The DIP/test switches come from the OSD (1 = on).
// Control rotation (M16: `rot_q` from nb1_orientation, which composes the
// game's orientation, the OSD display orientation and the OSD Controls
// option): the stick, taken in screen coordinates, is turned clockwise by
// rot_q quarter turns into the game's directions -- 1: screen-left -> game
// up, screen-up -> game right, screen-right -> game down, screen-down ->
// game left (M14's ROT90 "Rotated screen"); 2: opposite; 3: screen-right ->
// game up (M14's ROT270). 0 = MAME's mapping. Buttons are never turned.
// Everything is registered once in clk_sys (hps_io runs on clk_sys);
// nothing here is game-specific.
module nb1_inputs (
    input  wire        clk_sys,
    input  wire [31:0] joy0,          // hps_io joystick_0..3
    input  wire [31:0] joy1,
    input  wire [31:0] joy2,
    input  wire [31:0] joy3,
    input  wire        sw_service_mode,  // OSD: SW1:1 on
    input  wire        sw_freeze,        // OSD: SW1:2 on
    input  wire        sw_test,          // OSD: test switch on
    input  wire [1:0]  rot_q,            // stick turned CW by rot_q quarter turns (nb1_orientation)

    output reg  [7:0]  p1 = 8'hFF,
    output reg  [7:0]  p2 = 8'hFF,
    output reg  [7:0]  p3 = 8'hFF,
    output reg  [7:0]  p4 = 8'hFF,
    output reg  [7:0]  misc = 8'hFF
);
    // MiSTer order = NB-1 bit order: {start, b3, b2, b1, up, down, left, right}
    function automatic [7:0] player(input [31:0] j);
        reg r, l, d, u;
        begin
            case (rot_q)
                2'd0: {u, d, l, r} = j[3:0];
                2'd1: {u, d, l, r} = {j[1], j[0], j[2], j[3]};  // up=scr-left, down=scr-right, left=scr-down, right=scr-up
                2'd2: {u, d, l, r} = {j[2], j[3], j[0], j[1]};  // up=scr-down, down=scr-up, left=scr-right, right=scr-left
                2'd3: {u, d, l, r} = {j[0], j[1], j[3], j[2]};  // up=scr-right, down=scr-left, left=scr-up, right=scr-down
            endcase
            player = ~{j[7:4], u, d, l, r};
        end
    endfunction

    always @(posedge clk_sys) begin
        p1   <= player(joy0);
        p2   <= player(joy1);
        p3   <= player(joy2);
        p4   <= player(joy3);
        misc <= ~{ joy0[9] | joy1[9] | joy2[9] | joy3[9],   // 7 service
                   sw_test,                                // 6 test switch
                   joy0[8], joy1[8], joy2[8], joy3[8],     // 5..2 coin 1..4
                   sw_service_mode,                        // 1 SW1:1
                   sw_freeze };                            // 0 SW1:2
    end
endmodule
