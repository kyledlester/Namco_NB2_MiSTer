//============================================================================
//
//  Namco NB-2 MiSTer core -- top level (emu).
//  Copyright (C) 2026 Kyle Lester
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Structure follows MiSTer-devel/Template_MiSTer (Template.sv, GPL-2.0+) and the
//  author's Namco NB-1 core (NB1.sv).
//
//============================================================================
//
// MiSTer glue around rtl/nb2/nb2_core.sv (the NB-2 board): hps_io, the 96.768 MHz PLL, the OSD, CRT Adjust and
// the framework video/audio paths. Both NB-2 games are ROT0 288 x 224. The DDR3 port carries the sprite-graphics
// store (docs/ARCHITECTURE.md) and, for the OSD Orientation Vertical CW/CCW, screen_rotate's framebuffer writes.

module emu
(
	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;
assign AUDIO_S = 1;
assign LED_POWER = 0;
assign BUTTONS = 0;

// 288x224 on a 4:3 screen; turned 90 degrees (Vertical CW/CCW) it is 3:4.
wire [1:0] ar = status[122:121];
wire       video_rotated;
assign VIDEO_ARX = (!ar) ? (video_rotated ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (video_rotated ? 12'd4 : 12'd3) : 12'd0;

// Orientation (one OSD item, as the NB-1 core and the NA-1/NA-2 core), status[14:13]:
//   orient  OSD label      HDMI/scaler     native (analog)  no_rotate rotate_ccw osd_flip
//   00      Horizontal     H               H                1         x          0
//   01      Vertical CCW   H rot 90 CCW    H                0         1          0
//   10      Vertical CW    H rot 90 CW     H                0         0          0
//   11      Flipped        H rot 180       H rot 180        1         x          1
// Vertical CW/CCW are the framework's screen_rotate (DDR3 framebuffer -> HDMI scaler; Direct Video bypasses
// them); Flipped is made by the core's renderers (nb2_core osd_flip), so it reaches the 15 kHz output too.
wire [1:0] orient     = status[14:13];
wire       rot_vert   = (orient == 2'd1) || (orient == 2'd2);
wire       no_rotate  = !rot_vert || direct_video;
wire       rotate_ccw = (orient == 2'd1);
wire       osd_flip   = (orient == 2'd3);

`include "build_id.v"
// Status bits: 0 Reset, 1 Re-run ROM check, 4 ROM cache off (diagnostic), 5 Service mode, 10:9 Stereo mix,
// 12:11 Scandoubler Fx, 14:13 Orientation, 122:121 Aspect ratio; CRT Adjust as the NB-1 core: 96 On, 104:101 H-Position,
// 108:105 V-Shift, 116:112 H-Size (H1 hides the amounts while CRT Adjust is Off).
localparam CONF_STR = {
	"NB2;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[14:13],Orientation,Horizontal,Vertical CCW,Vertical CW,Flipped;",
	"O[12:11],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"-;",
	"P1,CRT Adjust;",
	"P1O[96],CRT Adjust,Off,On;",
	"H1P1O[116:112],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[104:101],CRT H-Position,0,+6,+12,+18,+24,+30,+36,+42,-48,-42,-36,-30,-24,-18,-12,-6;",
	"H1P1O[108:105],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"O[5],Service mode (SW1-1),Off,On;",
	"-;",
	"O[10:9],Stereo mix,None,25%,50%,100%;",
	"-;",
	"T[0],Reset;",
	"T[1],Re-run ROM check;",
	"R[0],Reset and close OSD;",
	"-;",
	"J1,Button 1,Button 2,Button 3,Start,Coin,Service;",
	"jn,A,B,X,Start,Select,R;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire         forced_scandoubler, direct_video;
wire  [21:0] gamma_bus;
wire   [1:0] buttons;
wire [127:0] status;
wire  [31:0] joystick_0, joystick_1, joystick_2, joystick_3;

wire        ioctl_download, ioctl_wr, ioctl_wait, ioctl_upload, ioctl_rd, ioctl_upload_req;
wire [15:0] ioctl_index, ioctl_dout, ioctl_din;
wire [26:0] ioctl_addr;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),
	.video_rotated(video_rotated),
	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~status[96], 1'b0}),
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(8'd1),
	.ioctl_din(ioctl_din),
	.ioctl_rd(ioctl_rd)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;      // 96.768 MHz = 2 x the 48.384 MHz NB-2 master oscillator
wire pll_locked;
nb1_pll pll (.refclk(CLK_50M), .rst(1'b0), .clk_sys(clk_sys), .locked(pll_locked));

///////////////////////   NB-2 BOARD   ///////////////////////////

reg pcache_bypass = 1'b0;
always @(posedge clk_sys) pcache_bypass <= status[4];

wire        ce_pix, hblank, vblank, hsync, vsync, line_end, frame_end, rom_loading, cpu_running;
wire [7:0]  core_r, core_g, core_b;
wire [8:0]  hcount, vcount;
wire signed [15:0] snd_l, snd_r;

nb2_core core
(
	.clk_sys(clk_sys),
	.pll_locked(pll_locked),
	.reset_request(RESET | status[0] | buttons[1]),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload),
	.ioctl_rd(ioctl_rd),
	.ioctl_din(ioctl_din),
	.ioctl_upload_req(ioctl_upload_req),
	.joy0(joystick_0), .joy1(joystick_1), .joy2(joystick_2), .joy3(joystick_3),
	.sw_service(status[5]),
	.rerun_check(status[1]),
	.pcache_bypass(pcache_bypass),
	.osd_flip(osd_flip),
	.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK),
	.DDRAM_BUSY(obj_busy), .DDRAM_BURSTCNT(), .DDRAM_ADDR(obj_addr), .DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(obj_rd), .DDRAM_DIN(obj_din), .DDRAM_BE(obj_be),
	.DDRAM_WE(obj_we),
	.ce_pix(ce_pix), .r(core_r), .g(core_g), .b(core_b), .hblank(hblank), .vblank(vblank), .hsync(hsync),
	.vsync(vsync), .hcount(hcount), .vcount(vcount), .line_end(line_end), .frame_end(frame_end),
	.audio_l(snd_l), .audio_r(snd_r),
	.rom_loading(rom_loading), .cpu_running(cpu_running), .chk_done(), .chk_pass(), .game_rot(),
	.dbg_cpu_pc(), .dbg_c75_pc(), .dbg_unmap_reads(), .dbg_tile_under(), .dbg_roz_under(), .dbg_spr_drops(), .dbg_wram_sdram()
);
assign DDRAM_CLK = clk_sys;

// DDR3: the sprite store (reads, ROM-load writes) and screen_rotate's framebuffer writes (Vertical CW/CCW)
wire        obj_busy, obj_rd, obj_we, sr_we;
wire [28:0] obj_addr, sr_addr;
wire [63:0] obj_din, sr_din;
wire  [7:0] obj_be, sr_be;
nb2_ddr_mux ddr_mux
(
	.clk(clk_sys),
	.a_rd(obj_rd), .a_we(obj_we), .a_addr(obj_addr), .a_din(obj_din), .a_be(obj_be), .a_busy(obj_busy),
	.b_we(sr_we), .b_addr(sr_addr), .b_din(sr_din), .b_be(sr_be),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DIN(DDRAM_DIN),
	.DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD),
	.b_overflows()
);

assign AUDIO_L   = snd_l;
assign AUDIO_R   = snd_r;
assign AUDIO_MIX = status[10:9];

///////////////////////   VIDEO   //////////////////////////////

wire        av_ce, av_hb, av_vb, av_hs, av_vs;
wire [23:0] av_rgb;
nb1_crt_adjust crt_adjust
(
	.clk_sys(clk_sys),
	.ce_pix(ce_pix),
	.frame_event(frame_end),
	.osd_on(status[96]),
	.osd_hsize(status[116:112]),
	.osd_hpos(status[104:101]),
	.osd_vshift(status[108:105]),
	.sd_off((status[12:11] == 2'd0) && !forced_scandoubler),
	.vb_next((vcount == 9'd263) ? 1'b0 : (vcount >= 9'd223)),
	.rgb_in({core_r, core_g, core_b}),
	.hblank_in(hblank),
	.vblank_in(vblank),
	.hsync_in(hsync),
	.vsync_in(vsync),
	.ce_out(av_ce),
	.rgb_out(av_rgb),
	.hblank_out(av_hb),
	.vblank_out(av_vb),
	.hsync_out(av_hs),
	.vsync_out(av_vs),
	.active(),
	.hsize_s(),
	.hpos_s(),
	.vsh_s()
);

arcade_video #(.WIDTH(288), .DW(24), .GAMMA(1)) arcade_video
(
	.clk_video(clk_sys),
	.ce_pix(av_ce),
	.RGB_in(av_rgb),
	.HBlank(av_hb),
	.VBlank(av_vb),
	.HSync(av_hs),
	.VSync(av_vs),
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_HS(VGA_HS),
	.VGA_VS(VGA_VS),
	.VGA_DE(VGA_DE),
	.VGA_SL(VGA_SL),
	.fx({1'b0, status[12:11]}),
	.forced_scandoubler(forced_scandoubler),
	.gamma_bus(gamma_bus)
);

// Orientation Vertical CW/CCW: the framework framebuffer rotation (HDMI/scaler only). Its DDR3 writes go through
// nb2_ddr_mux (above); Horizontal and Flipped keep it idle (FB_EN low).
screen_rotate screen_rotate
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_HS(VGA_HS),
	.VGA_VS(VGA_VS),
	.VGA_DE(VGA_DE),
	.rotate_ccw(rotate_ccw),
	.no_rotate(no_rotate),
	.flip(1'b0),
	.video_rotated(video_rotated),
	.FB_EN(FB_EN),
	.FB_FORMAT(FB_FORMAT),
	.FB_WIDTH(FB_WIDTH),
	.FB_HEIGHT(FB_HEIGHT),
	.FB_BASE(FB_BASE),
	.FB_STRIDE(FB_STRIDE),
	.FB_VBL(FB_VBL),
	.FB_LL(FB_LL),
	.DDRAM_CLK(),
	.DDRAM_BUSY(1'b0),
	.DDRAM_BURSTCNT(),
	.DDRAM_ADDR(sr_addr),
	.DDRAM_DIN(sr_din),
	.DDRAM_BE(sr_be),
	.DDRAM_WE(sr_we),
	.DDRAM_RD()
);
assign FB_FORCE_BLANK = 1'b0;

assign LED_DISK = {1'b0, rom_loading};
reg [5:0] blink = '0;
always @(posedge clk_sys) if (frame_end) blink <= blink + 6'd1;
assign LED_USER = cpu_running ? blink[5] : 1'b1;

endmodule
