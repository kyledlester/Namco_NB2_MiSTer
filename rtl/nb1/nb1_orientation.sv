// Namco NB-1 MiSTer core -- display orientation and control composition (M16; M17 native Flipped).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// One decode for everything the OSD Orientation selector (NA-1/NA-2 wording and order) changes,
// kept in its own module so sim/m16_orientation_tb.sv tests exactly what is built
// (docs/M16_PLAN.md, docs/M17_IMPLEMENTATION.md).
//
// Presentation. The NB-1 raster is always produced unrotated (the renderers, the M13 per-line raster
// effects and the native video timing are untouched).
//   - Vertical CW / CCW: the MiSTer framework's screen_rotate framebuffer (sys/arcade_video.v,
//     MISTER_FB -> DDR3 -> HDMI scaler): HDMI/scaler only; the analog output keeps H; MiSTer's global
//     Direct Video bypasses them (the HDMI then carries the native raster), as in the reference cores
//     and NA-1/NA-2.
//   - Flipped (M17): NATIVE, as on NA-1/NA-2 -- the 180-degree picture is made on the native raster by
//     nb1_native_flip (a DDR3 frame buffer: line 223's per-line C123/C116 state exists only at the end
//     of the frame), so it reaches the analog output, Direct Video and HDMI alike. screen_rotate's own
//     flip (M16, HDMI only) is no longer used.
//
//   orient  OSD label      HDMI/scaler     native     no_rotate rotate_ccw flip native_flip  d (CW)
//   00      Horizontal     H               H          1         x          0    0            0
//   01      Vertical CCW   H rot 90 CCW    H          0         1          0    0            3
//   10      Vertical CW    H rot 90 CW     H          0         0          0    0            1
//   11      Flipped        H rot 180       H rot 180  1         x          0    1            2
//   (Direct Video: no_rotate 1; Flipped stays native, so d = 2 there, else 0.)
//
// Controls. `game_rot` = the game's MAME orientation in quarter turns CW (board record v2 byte 13:
// ROT0 0, ROT90 1, ROT180 2, ROT270 3): the raster turned CW by game_rot shows the game upright,
// so the game's up points to screen direction -game_rot + d. With the OSD Controls option on
// Match display (ctl_match = 1), the stick is read in screen coordinates and turned CW by
// rot_q = (game_rot - d) mod 4 quarter turns into game directions (nb1_inputs), so pushing toward a
// screen side always moves toward it. With Game orientation (0) the stick is the game's own
// (rot_q = 0; MAME's mapping, e.g. a physically rotated monitor). For ROT90 on Horizontal this is
// exactly M14's "Rotated screen" mapping (screen-left -> game up); on Vertical CW it is 0.
// d is the transform of the picture the player sees: for CW/CCW that is the HDMI picture (the
// analog output stays H; a CRT user keeps Horizontal or physically rotates the set).
module nb1_orientation (
    input  wire [1:0] orient,        // OSD Orientation
    input  wire       direct_video,  // hps_io: MiSTer Direct Video
    input  wire [1:0] game_rot,      // game orientation, quarter turns CW
    input  wire       ctl_match,     // OSD Controls = Match display

    output wire       no_rotate,     // screen_rotate
    output wire       rotate_ccw,
    output wire       flip,          // screen_rotate 180: unused since M17 (0)
    output wire       native_flip,   // nb1_native_flip
    output wire [1:0] disp_q,        // transform of the shown picture, quarter turns CW
    output wire [1:0] rot_q          // nb1_inputs: stick turned CW by rot_q quarter turns
);
    wire vert = (orient == 2'd1) || (orient == 2'd2);
    assign no_rotate   = !vert || direct_video;
    assign rotate_ccw  = (orient == 2'd1);
    assign flip        = 1'b0;
    assign native_flip = (orient == 2'd3);
    assign disp_q      = (orient == 2'd3) ? 2'd2 :
                         direct_video     ? 2'd0 :
                         (orient == 2'd1) ? 2'd3 :
                         (orient == 2'd2) ? 2'd1 : 2'd0;
    assign rot_q       = ctl_match ? (game_rot - disp_q) : 2'd0;
endmodule
