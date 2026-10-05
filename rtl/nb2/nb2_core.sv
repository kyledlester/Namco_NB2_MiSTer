// Namco NB-2 MiSTer core -- the NB-2 board (everything below the MiSTer framework glue in NB2.sv).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Structure (docs/ARCHITECTURE.md): the NB-1 core's CPU / C75 / C352 / EEPROM / input / IRQ blocks on the
// NB-2 map (nb2_main_bus), the NB-2 memory system (nb2_sdram + nb2_memory for everything but sprite graphics,
// nb2_obj_store in DDR3 for those), the MRA records (nb2_board_config, nb2_rom_check), and the NB-2 video
// (C123 tiles, C169 ROZ, C355 sprites, NB-2 priority mixer, C116 palette).
//
// SDRAM clients (fixed priority, lowest first): 0 loader, 1 ROM checker, 2 68EC020 (ROM cache fills, SDRAM
// work RAM), 3 C75 data ROM + C352 voices, 4 C123 tile renderer, 5 ROZ renderer.
// DDR3 sprite store: port L = loader / checker (region OBJ), port S = C355 renderer.
module nb2_core #(
    parameter int  SLOTS = 28,              // block-RAM work-RAM pages
    parameter bit  VIDEO = 1'b1             // 0: no renderers (system benches of CPU/C75 only)
) (
    input  wire        clk_sys,             // 96.768 MHz
    input  wire        pll_locked,
    input  wire        reset_request,       // OSD/user reset, download in progress

    // hps_io
    input  wire        ioctl_download,
    input  wire [15:0] ioctl_index,
    input  wire        ioctl_wr,
    input  wire [26:0] ioctl_addr,
    input  wire [15:0] ioctl_dout,
    output wire        ioctl_wait,
    input  wire        ioctl_upload,
    input  wire        ioctl_rd,
    output wire [15:0] ioctl_din,
    output wire        ioctl_upload_req,

    input  wire [31:0] joy0, joy1, joy2, joy3,
    input  wire        sw_service,
    input  wire        rerun_check,
    input  wire        pcache_bypass,
    input  wire        osd_flip,            // OSD Orientation "Flipped": the whole picture turned 180 degrees

    // SDRAM
    inout  wire [15:0] SDRAM_DQ,
    output wire [12:0] SDRAM_A,
    output wire        SDRAM_DQML,
    output wire        SDRAM_DQMH,
    output wire [1:0]  SDRAM_BA,
    output wire        SDRAM_nCS,
    output wire        SDRAM_nWE,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_CKE,
    output wire        SDRAM_CLK,

    // DDR3 (MiSTer DDRAM port, clocked by clk_sys)
    input  wire        DDRAM_BUSY,
    output wire [7:0]  DDRAM_BURSTCNT,
    output wire [28:0] DDRAM_ADDR,
    input  wire [63:0] DDRAM_DOUT,
    input  wire        DDRAM_DOUT_READY,
    output wire        DDRAM_RD,
    output wire [63:0] DDRAM_DIN,
    output wire [7:0]  DDRAM_BE,
    output wire        DDRAM_WE,

    // video (native raster, 384 x 264 at ce_pix, 288 x 224 active)
    output wire        ce_pix,
    output wire [7:0]  r, g, b,
    output wire        hblank, vblank, hsync, vsync,
    output wire [8:0]  hcount, vcount,
    output wire        line_end, frame_end,

    // audio (C352 front L/R, 84 kHz, signed)
    output wire signed [15:0] audio_l,
    output wire signed [15:0] audio_r,

    // status
    output wire        rom_loading,
    output wire        cpu_running,
    output wire        chk_done,
    output wire        chk_pass,
    output wire [1:0]  game_rot,
    // diagnostics for the benches
    output wire [23:0] dbg_cpu_pc,
    output wire [23:0] dbg_c75_pc,
    output wire [31:0] dbg_unmap_reads,
    output wire [15:0] dbg_tile_under,     // C123 lines abandoned at their deadline (saturating)
    output wire [15:0] dbg_roz_under,      // ROZ lines abandoned
    output wire [15:0] dbg_spr_drops,      // C355 sprite lines abandoned
    output wire [31:0] dbg_wram_sdram      // 68EC020 work-RAM cycles served from SDRAM (cold pages)
);
    import nb2_mem_pkg::*;

    localparam [15:0] IOCTL_ROM = 16'd0, IOCTL_NVRAM = 16'd1, IOCTL_BOARD = 16'd2, IOCTL_CHECK = 16'd3;

    // ------------------------------------------------------------------ clocks, raster, reset
    wire ce_xtal, ce_cpu, ce_c352, ce_c75;
    wire [5:0] ce_phase;
    nb1_clock_enables clock_enables (.clk_sys(clk_sys), .ce_xtal(ce_xtal), .ce_cpu(ce_cpu), .ce_c352(ce_c352),
                                     .ce_c75(ce_c75), .ce_pix(ce_pix), .phase(ce_phase));
    wire de, vblank_begin;
    wire [8:0] c116_h, c116_v;
    nb1_video_timing video_timing (
        .clk_sys(clk_sys), .ce_pix(ce_pix), .hcount(hcount), .vcount(vcount), .c116_h(c116_h), .c116_v(c116_v),
        .hblank(hblank), .vblank(vblank), .de(de), .hsync(hsync), .vsync(vsync), .line_end(line_end),
        .frame_end(frame_end), .vblank_begin(vblank_begin));
    wire cpu_line_start, cpu_vbl, cpu_frame;
    wire [8:0] cpu_line_next;
    nb1_cpu_raster #(.LEAD(2)) cpu_raster (.line_end(line_end), .vcount(vcount), .line_start(cpu_line_start),
                                           .line_next(cpu_line_next), .vbl(cpu_vbl), .frame(cpu_frame));
    wire reset_sys, reset_cpu, reset_c75, reset_video;
    nb1_reset reset_gen (.clk_sys(clk_sys), .reset_request(reset_request | ioctl_download), .pll_locked(pll_locked),
                         .frame_end(frame_end), .reset_sys(reset_sys), .reset_cpu(reset_cpu),
                         .reset_c75(reset_c75), .reset_video(reset_video));
    wire mem_init = ~pll_locked;

    // ------------------------------------------------------------------ SDRAM
    localparam int NC = 6;
    localparam int TW = $clog2(NC) + 2;
    wire [NC-1:0]    m_valid, m_ready, m_we, m_rsp_valid;
    wire [NC*4-1:0]  m_region, m_be;
    wire [NC*26-1:0] m_offset;
    wire [NC*32-1:0] m_wdata;
    wire [31:0] m_rdata;
    wire [63:0] m_line;
    wire        m_err;
    wire sd_valid, sd_ready, sd_we, sd_rsp_valid, sd_rsp_write, sd_up;
    wire [24:1] sd_addr; wire [15:0] sd_wdata; wire [1:0] sd_be; wire [TW-1:0] sd_tag, sd_rsp_tag; wire [63:0] sd_rsp_line;
    nb2_sdram #(.TW(TW)) sdram (
        .clk(clk_sys), .init(mem_init),
        .req_valid(sd_valid), .req_ready(sd_ready), .req_we(sd_we), .req_addr(sd_addr), .req_wdata(sd_wdata),
        .req_be(sd_be), .req_tag(sd_tag), .rsp_valid(sd_rsp_valid), .rsp_write(sd_rsp_write), .rsp_tag(sd_rsp_tag),
        .rsp_line(sd_rsp_line), .ready(sd_up),
        .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_BA(SDRAM_BA), .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK));
    nb2_memory #(.CLIENTS(NC), .OUT_MAX(2), .TW(TW)) memory (
        .clk_sys(clk_sys), .init(mem_init),
        .req_valid(m_valid), .req_ready(m_ready), .req_region(m_region), .req_offset(m_offset), .req_we(m_we),
        .req_wdata(m_wdata), .req_be(m_be), .rsp_valid(m_rsp_valid), .rsp_rdata(m_rdata), .rsp_line(m_line),
        .rsp_err(m_err),
        .sd_valid(sd_valid), .sd_ready(sd_ready), .sd_we(sd_we), .sd_addr(sd_addr), .sd_wdata(sd_wdata),
        .sd_be(sd_be), .sd_tag(sd_tag), .sd_rsp_valid(sd_rsp_valid), .sd_rsp_write(sd_rsp_write),
        .sd_rsp_tag(sd_rsp_tag), .sd_rsp_line(sd_rsp_line), .busy_clients());

    // ------------------------------------------------------------------ DDR3 sprite store
    wire        ol_valid, ol_ready, ol_rsp_valid, ol_rsp_err, ol_we;
    wire [25:0] ol_offset;
    wire [31:0] ol_wdata, ol_rdata;
    wire [3:0]  ol_be;
    wire [63:0] ol_line;
    wire        os_valid, os_ready, os_rsp_valid;
    wire [25:0] os_offset;
    wire [63:0] os_line;
    // The store's reset drops queued sprite reads; it refuses new requests while asserted, so it must not follow
    // the download-held reset_video (the loader's OBJ writes would wait forever: hardware test 1).
    nb2_obj_store obj_store (
        .clk_sys(clk_sys), .reset(reset_video & ~ioctl_download),
        .l_valid(ol_valid), .l_ready(ol_ready), .l_offset(ol_offset), .l_we(ol_we), .l_wdata(ol_wdata), .l_be(ol_be),
        .l_rsp_valid(ol_rsp_valid), .l_rsp_rdata(ol_rdata), .l_rsp_line(ol_line), .l_rsp_err(ol_rsp_err),
        .s_valid(os_valid), .s_ready(os_ready), .s_offset(os_offset), .s_rsp_valid(os_rsp_valid), .s_rsp_line(os_line),
        .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT),
        .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE),
        .DDRAM_WE(DDRAM_WE), .max_latency());

    // ------------------------------------------------------------------ loader, checker, board record
    wire        ld_wait;
    wire        rom_loaded, ld_ovr, ld_rng, ld_wrerr, ld_ovalid, ld_we;
    wire [26:0] stream_bytes;
    wire [3:0]  ld_region, ld_be;
    wire [25:0] ld_offset;
    wire [31:0] ld_wdata;
    reg         ld_obj_out = 1'b0;            // the loader's outstanding request is in the sprite store
    wire        ck_ovalid, ck_we;
    wire [3:0]  ck_region, ck_be;
    wire [25:0] ck_offset;
    wire [31:0] ck_wdata;
    always @(posedge clk_sys) begin
        if (ld_ovalid && ol_ready) ld_obj_out <= 1'b1;
        else if (ol_rsp_valid)     ld_obj_out <= 1'b0;
    end
    assign ol_valid  = ld_ovalid || ck_ovalid;
    assign ol_offset = ld_ovalid ? ld_offset : ck_offset;
    assign ol_we     = ld_ovalid && ld_we;
    assign ol_wdata  = ld_wdata;
    assign ol_be     = ld_ovalid ? ld_be : 4'hF;

    nb2_rom_loader #(.INDEX(IOCTL_ROM)) rom_loader (
        .clk_sys(clk_sys), .init(mem_init),
        .ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
        .ioctl_dout(ioctl_dout), .ioctl_wait(ld_wait),
        .sd_valid(m_valid[0]), .sd_ready(m_ready[0]), .sd_rsp_valid(m_rsp_valid[0]),
        .obj_valid(ld_ovalid), .obj_ready(ol_ready && ld_ovalid), .obj_rsp_valid(ol_rsp_valid && ld_obj_out),
        .req_region(ld_region), .req_offset(ld_offset), .req_we(ld_we), .req_wdata(ld_wdata), .req_be(ld_be),
        .rsp_err(m_rsp_valid[0] ? m_err : ol_rsp_err),
        .loading(rom_loading), .rom_loaded(rom_loaded), .stream_bytes(stream_bytes),
        .err_overrun(ld_ovr), .err_range(ld_rng), .err_write(ld_wrerr));
    assign m_region[0*4 +: 4] = ld_region; assign m_offset[0*26 +: 26] = ld_offset; assign m_we[0] = ld_we;
    assign m_wdata[0*32 +: 32] = ld_wdata; assign m_be[0*4 +: 4] = ld_be;

    wire chk_ok, chk_running, chk_len;
    wire [5:0] chk_n; wire [63:0] chk_st; wire [3:0] chk_runs;
    nb2_rom_check #(.INDEX(IOCTL_CHECK), .MAX_ENTRIES(32)) rom_check (
        .clk_sys(clk_sys), .init(mem_init), .reset(reset_sys), .restart(rerun_check),
        .ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
        .ioctl_dout(ioctl_dout), .rom_loaded(rom_loaded), .stream_bytes(stream_bytes),
        .sd_valid(m_valid[1]), .sd_ready(m_ready[1]), .sd_rsp_valid(m_rsp_valid[1]), .sd_rsp_rdata(m_rdata),
        .sd_rsp_err(m_err),
        .obj_valid(ck_ovalid), .obj_ready(ol_ready && !ld_ovalid), .obj_rsp_valid(ol_rsp_valid && !ld_obj_out),
        .obj_rsp_rdata(ol_rdata), .obj_rsp_err(ol_rsp_err),
        .req_region(ck_region), .req_offset(ck_offset), .req_we(ck_we), .req_wdata(ck_wdata), .req_be(ck_be),
        .record_ok(chk_ok), .entry_count(chk_n), .entry_status(chk_st), .running(chk_running), .done(chk_done),
        .all_pass(chk_pass), .length_ok(chk_len), .run_count(chk_runs));
    assign m_region[1*4 +: 4] = ck_region; assign m_offset[1*26 +: 26] = ck_offset; assign m_we[1] = ck_we;
    assign m_wdata[1*32 +: 32] = ck_wdata; assign m_be[1*4 +: 4] = ck_be;

    wire brd_ok, brd_seen;
    wire [7:0] kc_mode; wire [3:0] kc_idw, kc_rw, xform; wire [15:0] kc_id; wire [56:0] hot_pages;
    nb2_board_config #(.INDEX(IOCTL_BOARD)) board_config (
        .clk_sys(clk_sys), .init(mem_init), .ioctl_download(ioctl_download), .ioctl_index(ioctl_index),
        .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
        .record_ok(brd_ok), .record_seen(brd_seen), .kc_mode(kc_mode), .kc_id_word(kc_idw), .kc_id(kc_id),
        .kc_rnd_word(kc_rw), .xform(xform), .game_rot(game_rot), .hot_pages(hot_pages));

    // C75 internal ROM: written from the stream as it passes (BIOS window, nb2_mem_pkg)
    wire c75_irom_we = ioctl_download && (ioctl_index == IOCTL_ROM) && ioctl_wr &&
                       (ioctl_addr >= 27'h1E80000) && (ioctl_addr < 27'h1E84000);

    // ------------------------------------------------------------------ 68EC020
    reg cpu_go = 1'b0;
    always @(posedge clk_sys) begin
        if (reset_cpu | ~rom_loaded)                                    cpu_go <= 1'b0;
        else if (cpu_frame && !chk_running && (chk_done || !chk_ok))    cpu_go <= 1'b1;
    end
    wire cpu_reset = reset_cpu | ~cpu_go;

    wire        bc_start, bc_uds, bc_lds, bc_done, pd_sample;
    wire [23:1] bc_addr, pd_addr;
    wire [1:0]  bc_type;
    wire [2:0]  bc_fc, cpu_ipl;
    wire [15:0] bc_wdata, bc_rdata;
    wire [23:0] fetch_pc;
    nb1_cpu #(.MIN_GAP(3), .SAMPLE_DELAY(2), .CREDIT_MAX(32)) cpu (
        .clk_sys(clk_sys), .reset(cpu_reset), .ce_cpu(ce_cpu), .frame_tick(frame_end), .ipl(cpu_ipl),
        .bc_start(bc_start), .bc_addr(bc_addr), .bc_type(bc_type), .bc_fc(bc_fc), .bc_uds(bc_uds), .bc_lds(bc_lds),
        .bc_wdata(bc_wdata), .bc_done(bc_done), .bc_rdata(bc_rdata), .pd_sample(pd_sample), .pd_addr(pd_addr),
        .running(cpu_running), .fetch_pc(fetch_pc), .vec_ssp(), .vec_pc(), .vec_seen(), .steps(), .bus_cycles(),
        .steps_last_frame(), .earned_last_frame(), .lost_credits(), .max_mem_stall(), .mem_stall_cycles(),
        .cacr(), .vbr(), .credit());
    assign dbg_cpu_pc = fetch_pc;

    // EEPROM persistence (MiSTer NVRAM, ioctl index 1)
    wire nv_mem_en, nv_mem_we, eep_cpu_write, nv_wait;
    wire [10:1] nv_mem_addr;
    wire [15:0] nv_mem_wdata, nv_mem_rdata;
    nb1_nvram #(.INDEX(IOCTL_NVRAM), .BYTES(2048)) nvram (
        .clk_sys(clk_sys), .ioctl_download(ioctl_download), .ioctl_upload(ioctl_upload), .ioctl_index(ioctl_index),
        .ioctl_wr(ioctl_wr), .ioctl_rd(ioctl_rd), .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
        .ioctl_din(ioctl_din), .ioctl_wait(nv_wait), .upload_req(ioctl_upload_req),
        .mem_en(nv_mem_en), .mem_we(nv_mem_we), .mem_addr(nv_mem_addr), .mem_wdata(nv_mem_wdata),
        .mem_rdata(nv_mem_rdata), .cpu_write(eep_cpu_write), .dirty(), .dirty_events(), .loads(), .saves(),
        .last_bytes(), .last_action());
    assign ioctl_wait = ld_wait | nv_wait;

    // video read side of the bus
    wire [14:0] vid_vram_addr;
    wire [15:0] vid_vram_data;
    wire [12:0] vid_pen;
    wire [23:0] vid_rgb;
    wire [127:0] c116_regs;
    wire [511:0] c123_ctl;
    wire [15:0] roz_vram_addr, roz_vram_data;
    wire [255:0] roz_ctl;
    wire [63:0]  spr_bank;
    wire [127:0] tile_bank, roz_bank;
    wire [14:0] obj_snap_addr;
    wire [15:0] obj_snap_data;
    wire [63:0] obj_pos;
    wire        obj_capture;
    wire        c75sh_en, c75sh_we, c75_run, c75_restart;
    wire [13:0] c75sh_addr;
    wire [15:0] c75sh_wdata, c75sh_rdata;
    wire [1:0]  c75sh_be;
    wire [3:0]  vbl_level, pos_level;

    nb2_main_bus #(.LINES_LOG2(1), .PCACHE_LOG2(9), .SLOTS(SLOTS)) main_bus (
        .clk_sys(clk_sys), .reset(cpu_reset), .pd_sample(pd_sample), .pd_addr(pd_addr), .pcache_bypass(pcache_bypass),
        .bc_start(bc_start), .bc_addr(bc_addr), .bc_type(bc_type), .bc_fc(bc_fc), .bc_uds(bc_uds), .bc_lds(bc_lds),
        .bc_wdata(bc_wdata), .bc_done(bc_done), .bc_rdata(bc_rdata),
        .mreq_valid(m_valid[2]), .mreq_ready(m_ready[2]), .mreq_region(m_region[2*4 +: 4]),
        .mreq_offset(m_offset[2*26 +: 26]), .mreq_we(m_we[2]), .mreq_wdata(m_wdata[2*32 +: 32]),
        .mreq_be(m_be[2*4 +: 4]), .mrsp_valid(m_rsp_valid[2]), .mrsp_rdata(m_rdata), .mrsp_line(m_line),
        .mrsp_err(m_err),
        .hot_pages(hot_pages), .kc_cfg_valid(brd_ok), .kc_cfg_mode(kc_mode), .kc_cfg_id_word(kc_idw),
        .kc_cfg_id(kc_id), .kc_cfg_rnd_word(kc_rw),
        .c75sh_en(c75sh_en), .c75sh_we(c75sh_we), .c75sh_addr(c75sh_addr), .c75sh_wdata(c75sh_wdata),
        .c75sh_be(c75sh_be), .c75sh_rdata(c75sh_rdata), .c75_run(c75_run), .c75_restart(c75_restart),
        .vbl_event(cpu_vbl), .pos_event(cpu_line_start), .pos_line(cpu_line_next), .ipl(cpu_ipl),
        .vbl_level(vbl_level), .pos_level(pos_level),
        .nv_en(nv_mem_en), .nv_we(nv_mem_we), .nv_addr(nv_mem_addr), .nv_wdata(nv_mem_wdata),
        .nv_rdata(nv_mem_rdata), .eep_cpu_write(eep_cpu_write),
        .vid_vram_addr(vid_vram_addr), .vid_vram_data(vid_vram_data), .vid_pen(vid_pen), .vid_rgb(vid_rgb),
        .c116_regs(c116_regs), .c123_ctl(c123_ctl), .roz_vram_addr(roz_vram_addr), .roz_vram_data(roz_vram_data),
        .roz_ctl(roz_ctl), .spr_bank(spr_bank), .tile_bank(tile_bank), .roz_bank(roz_bank),
        .obj_snap_addr(obj_snap_addr), .obj_snap_data(obj_snap_data), .obj_pos(obj_pos), .obj_hold(obj_capture),
        .cls_seen(), .unmap_reads(dbg_unmap_reads), .unmap_writes(), .last_unmap(), .wram_sdram_cycles(dbg_wram_sdram),
        .line_hits(), .line_misses(), .fill_error());

    // ------------------------------------------------------------------ C75 + C352
    wire [7:0] in_p1, in_p2, in_p3, in_p4, in_misc;
    nb1_inputs inputs (
        .clk_sys(clk_sys), .joy0(joy0), .joy1(joy1), .joy2(joy2), .joy3(joy3),
        .sw_service_mode(sw_service), .sw_freeze(1'b0), .sw_test(1'b0), .rot_q(2'd0),
        .p1(in_p1), .p2(in_p2), .p3(in_p3), .p4(in_p4), .misc(in_misc));
    wire c75_irq0, c75_irq2;
    nb1_c75_irq c75_irq (.clk_sys(clk_sys), .irq0(c75_irq0), .irq2(c75_irq2), .ticks());

    wire        c75_mreq_valid, c75_mreq_ready, c75_mrsp_valid;
    wire [3:0]  c75_mreq_region;
    wire [24:0] c75_mreq_offset;
    wire        c352_req, c352_we, c352_ack;
    wire [10:0] c352_addr;
    wire [1:0]  c352_be;
    wire [15:0] c352_wdata, c352_rdata;
    wire        snd_mreq_valid, snd_mreq_ready, snd_mrsp_valid;
    wire [24:0] snd_mreq_offset;
    wire [3:0]  snd_mreq_region;
    reg  [10:0] snd_div = 11'd0;
    reg         snd_tick = 1'b0;
    always @(posedge clk_sys) begin
        snd_tick <= (snd_div == 11'd1151);
        snd_div  <= (snd_div == 11'd1151) ? 11'd0 : snd_div + 11'd1;
    end
    nb2_c352 c352 (
        .clk_sys(clk_sys), .reset(cpu_reset), .sample_tick(snd_tick),
        .bus_req(c352_req), .bus_we(c352_we), .bus_addr(c352_addr), .bus_be(c352_be), .bus_wdata(c352_wdata),
        .bus_ack(c352_ack), .bus_rdata(c352_rdata),
        .mreq_valid(snd_mreq_valid), .mreq_ready(snd_mreq_ready), .mreq_offset(snd_mreq_offset),
        .mreq_region(snd_mreq_region), .mrsp_valid(snd_mrsp_valid), .mrsp_line(m_line), .mrsp_err(m_err),
        .out_l(audio_l), .out_r(audio_r), .out_valid(), .overruns(), .fetches(), .busy_voices(), .max_wait());
    wire [24:0] mx_offset;
    nb1_mem_mux2 snd_mux (
        .clk_sys(clk_sys),
        .a_valid(c75_mreq_valid), .a_ready(c75_mreq_ready), .a_region(c75_mreq_region), .a_offset(c75_mreq_offset),
        .a_rsp_valid(c75_mrsp_valid),
        .b_valid(snd_mreq_valid), .b_ready(snd_mreq_ready), .b_region(snd_mreq_region), .b_offset(snd_mreq_offset),
        .b_rsp_valid(snd_mrsp_valid),
        .m_valid(m_valid[3]), .m_ready(m_ready[3]), .m_region(m_region[3*4 +: 4]), .m_offset(mx_offset),
        .m_rsp_valid(m_rsp_valid[3]));
    assign m_offset[3*26 +: 26] = {1'b0, mx_offset};

    reg c75_reset_q = 1'b1;
    always @(posedge clk_sys) c75_reset_q <= cpu_reset | ~c75_run | c75_restart;
    nb1_c75 c75 (
        .clk_sys(clk_sys), .reset(c75_reset_q), .ce_c75(ce_c75),
        .irom_we(c75_irom_we), .irom_waddr(ioctl_addr[13:1]), .irom_wdata(ioctl_dout),
        .sh_en(c75sh_en), .sh_we(c75sh_we), .sh_addr(c75sh_addr), .sh_wdata(c75sh_wdata), .sh_be(c75sh_be),
        .sh_rdata(c75sh_rdata),
        .mreq_valid(c75_mreq_valid), .mreq_ready(c75_mreq_ready), .mreq_region(c75_mreq_region),
        .mreq_offset(c75_mreq_offset), .mreq_we(m_we[3]), .mreq_wdata(m_wdata[3*32 +: 32]), .mreq_be(m_be[3*4 +: 4]),
        .mrsp_valid(c75_mrsp_valid), .mrsp_line(m_line), .mrsp_err(m_err),
        .c352_req(c352_req), .c352_we(c352_we), .c352_addr(c352_addr), .c352_be(c352_be), .c352_wdata(c352_wdata),
        .c352_ack(c352_ack), .c352_rdata(c352_rdata),
        .irq0_in(c75_irq0), .irq2_in(c75_irq2),
        .in_p1(in_p1), .in_p2(in_p2), .in_p3(in_p3), .in_p4(in_p4), .in_misc(in_misc),
        .running(), .pc(dbg_c75_pc), .instr_count(), .overrun_count(), .int0_count(), .int2_count(),
        .drom_reads(), .drom_misses(), .max_drom_wait(), .c352_writes(), .unmapped_acc(), .hs_sets(), .hs_a000(),
        .hs_a0a0(), .bus_req_o(), .bus_we_o(), .bus_addr_o(), .bus_be_o(), .bus_wdata_o(), .bus_ack_o(),
        .bus_rdata_o(), .irq_ack_o(), .irq_ack_line_o());

    // ------------------------------------------------------------------ video
    // C123 tiles (SDRAM client 4), C169 ROZ (client 5), C355 sprites (DDR3 store port S), NB-2 mixer + C116.
    // The renderers are held idle while the CPU is held (as NB-1), so the ROM check never competes with them.
    // C355 position and bank are sampled at the CPU vblank (NB-1 M13); the snapshot/frame switch stay at the video
    // vblank.
    // OSD Flipped (180 degrees) is native: every renderer draws display line D from logical line 223 - D (into
    // the line-buffer slot of D), and the output reads x from 287 - x. Every line of a frame is drawn from the
    // state the game wrote in the previous vblank (the VBL handler runs after the last line's state is latched),
    // so drawing the lines bottom-up gives exactly the turned picture. oflip changes only between frames.
    reg oflip = 1'b0;
    always @(posedge clk_sys) if (vblank_begin) oflip <= osd_flip;

    generate if (VIDEO) begin : g_video
        wire        rnd_disp_rd, rnd_disp_buf;
        wire [8:0]  rnd_disp_x;
        wire [15:0] rnd_disp_entry;
        wire        rnd_line_busy;
        nb2_c123_render tiles (
            .clk_sys(clk_sys), .reset(cpu_reset), .line_end(line_end), .vcount(vcount), .oflip(oflip), .ctl(c123_ctl),
            .xform(xform[1:0]), .tbank(tile_bank[63:0]),
            .vram_addr(vid_vram_addr), .vram_data(vid_vram_data),
            .mreq_valid(m_valid[4]), .mreq_ready(m_ready[4]), .mreq_region(m_region[4*4 +: 4]),
            .mreq_offset(m_offset[4*26 +: 26]), .mreq_we(m_we[4]), .mreq_wdata(m_wdata[4*32 +: 32]),
            .mreq_be(m_be[4*4 +: 4]), .mrsp_valid(m_rsp_valid[4]), .mrsp_line(m_line), .mrsp_err(m_err),
            .disp_rd(rnd_disp_rd), .disp_buf(rnd_disp_buf), .disp_x(rnd_disp_x), .disp_entry(rnd_disp_entry),
            .layer_mask(), .flip_out(), .chr_frame(), .shape_frame(), .worst_frame(), .underruns(dbg_tile_under),
            .line_busy(rnd_line_busy), .lines_done(), .mem_error());

        wire [16:0] roz_e0, roz_e1;
        nb2_roz_render roz (
            .clk_sys(clk_sys), .reset(cpu_reset), .line_end(line_end), .vcount(vcount), .oflip(oflip), .ctl(roz_ctl),
            .rbank(roz_bank), .swap46(xform[2]), .vram_addr(roz_vram_addr), .vram_data(roz_vram_data),
            .mreq_valid(m_valid[5]), .mreq_ready(m_ready[5]), .mreq_region(m_region[5*4 +: 4]),
            .mreq_offset(m_offset[5*26 +: 26]), .mreq_we(m_we[5]), .mreq_wdata(m_wdata[5*32 +: 32]),
            .mreq_be(m_be[5*4 +: 4]), .mrsp_valid(m_rsp_valid[5]), .mrsp_line(m_line), .mrsp_err(m_err),
            .disp_rd(rnd_disp_rd), .disp_buf(rnd_disp_buf), .disp_x(rnd_disp_x), .disp_e0(roz_e0), .disp_e1(roz_e1),
            .reads_frame(), .worst_frame(), .underruns(dbg_roz_under), .mem_error());

        reg [63:0] obj_pos_v = '0;
        reg [63:0] spr_bank_v = '0;
        always @(posedge clk_sys) if (cpu_vbl) begin
            obj_pos_v  <= obj_pos;
            spr_bank_v <= spr_bank;
        end
        wire        spr_rd;
        wire [1:0]  spr_slot;
        wire [8:0]  spr_x;
        wire [15:0] spr_entry;
        nb2_c355_render sprites (
            .clk_sys(clk_sys), .reset(cpu_reset), .line_end(line_end), .vblank_begin(vblank_begin),
            .zoom_cont(1'b0), .vcount(vcount), .oflip(oflip),
            .snap_addr(obj_snap_addr), .snap_data(obj_snap_data), .pos_in(obj_pos_v), .bank_in(spr_bank_v),
            .swap12(xform[3]), .capture_busy(obj_capture),
            .mreq_valid(os_valid), .mreq_ready(os_ready), .mreq_region(), .mreq_offset(os_offset), .mreq_we(),
            .mreq_wdata(), .mreq_be(), .mrsp_valid(os_rsp_valid), .mrsp_line(os_line), .mrsp_err(1'b0),
            .disp_rd(spr_rd), .disp_slot(spr_slot), .disp_x(spr_x), .disp_entry(spr_entry),
            .spr_count(), .cell_count(), .overflow(), .drops(dbg_spr_drops), .drop_frames(), .drop_max(), .fetch_frame(),
            .mem_error());

        nb2_video_out video_out (
            .clk_sys(clk_sys), .ce_pix(ce_pix), .hcount(hcount), .vcount(vcount), .de(de), .line_end(line_end), .oflip(oflip),
            .disp_rd(rnd_disp_rd), .disp_buf(rnd_disp_buf), .disp_x(rnd_disp_x), .disp_entry(rnd_disp_entry),
            .roz_e0(roz_e0), .roz_e1(roz_e1),
            .spr_rd(spr_rd), .spr_slot(spr_slot), .spr_x(spr_x), .spr_entry(spr_entry),
            .vid_pen(vid_pen), .vid_rgb(vid_rgb), .c116_regs(c116_regs), .r(r), .g(g), .b(b));
    end else begin : g_novideo
        assign m_valid[4] = 1'b0; assign m_region[4*4 +: 4] = '0; assign m_offset[4*26 +: 26] = '0; assign m_we[4] = 1'b0;
        assign m_wdata[4*32 +: 32] = '0; assign m_be[4*4 +: 4] = '0;
        assign m_valid[5] = 1'b0; assign m_region[5*4 +: 4] = '0; assign m_offset[5*26 +: 26] = '0; assign m_we[5] = 1'b0;
        assign m_wdata[5*32 +: 32] = '0; assign m_be[5*4 +: 4] = '0;
        assign os_valid = 1'b0; assign os_offset = '0;
        assign vid_vram_addr = '0; assign vid_pen = '0; assign roz_vram_addr = '0; assign obj_snap_addr = '0;
        assign obj_capture = 1'b0;
        assign r = 8'd0; assign g = 8'd0; assign b = 8'd0;
        assign dbg_tile_under = '0; assign dbg_roz_under = '0; assign dbg_spr_drops = '0;
    end endgenerate
endmodule
