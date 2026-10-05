// Namco NB-2 MiSTer core -- 68EC020 main bus.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The NB-1 core's nb1_main_bus (docs/PROVENANCE.md) for the NB-2 map (nb2_cpu_map_pkg, MAME 0.289
// namconb2_state::maincpu_am; docs/NB2_HARDWARE_REFERENCE.md section 2). Unchanged from NB-1: the bus-cycle
// contract with nb1_cpu (bc_start pulse; bc_done combinational for no-wait cycles, then held), the M22 predecode
// (class and RAM index registered on pd_sample), registered device read data (SYNC_REG), the two-line program
// buffer (L0) with the ROM cache behind it (L1), the shared-RAM write lock, the sprite-RAM write hold during the
// renderer's vblank capture, the NB-1 devices (C116, C123 VRAM/control, C355 RAM/position, EEPROM, KEYCUS, IRQ).
// NB-2:
//   ROM      $000000-$0FFFFF program and $400000-$4FFFFF data ROM, one cache (nb2_rom_cache; addr[20] = data).
//   WRAM     $208000-$2FFFFF and $1C0000-$1CFFFF: the hot 4 KiB pages of $208000-$240FFF named by the board
//            record live in block RAM (SLOTS pages, in page order); every other address is backed by SDRAM
//            region WRAM through this bus's memory port (one access per bus cycle; correct, only slower).
//   cpureg   $F00000 VBL level, $F00002 POS level, $F00004 VBL ack, $F00006 POS ack, $F00016 C75 control
//            (even bytes); reads $FF [MAME-SOURCE namconb2_state::cpureg_w/r].
//   REGS     $640000-$64000F RAM, sprite bank $900008-$90000F (8 bytes), tile bank $940000-$94000F, ROZ bank
//            $980000-$98000F (16 bytes each), all plain RAM read back as written.
//   C169     ROZ VRAM $700000-$71FFFF (64K words, dual-port: renderer port) and control $740000-$74001F.
//   unmapped reads $0000 (MAME), writes dropped; counted.
module nb2_main_bus #(
    parameter int LINES_LOG2 = 1,
    parameter int PCACHE_LOG2 = 9,
    parameter int SLOTS = 28                    // block-RAM work-RAM pages (4 KiB each)
) (
    input  wire        clk_sys,
    input  wire        reset,
    input  wire        pd_sample,
    input  wire [23:1] pd_addr,
    input  wire        pcache_bypass,

    input  wire        bc_start,
    input  wire [23:1] bc_addr,
    input  wire [1:0]  bc_type,
    input  wire [2:0]  bc_fc,
    input  wire        bc_uds,
    input  wire        bc_lds,
    input  wire [15:0] bc_wdata,
    output wire        bc_done,
    output reg  [15:0] bc_rdata,

    // nb2_memory client (ROM fills, SDRAM-backed work RAM)
    output wire        mreq_valid,
    input  wire        mreq_ready,
    output wire [3:0]  mreq_region,
    output wire [25:0] mreq_offset,
    output wire        mreq_we,
    output wire [31:0] mreq_wdata,
    output wire [3:0]  mreq_be,
    input  wire        mrsp_valid,
    input  wire [31:0] mrsp_rdata,
    input  wire [63:0] mrsp_line,
    input  wire        mrsp_err,

    // board record
    input  wire [56:0] hot_pages,
    input  wire        kc_cfg_valid,
    input  wire [7:0]  kc_cfg_mode,
    input  wire [3:0]  kc_cfg_id_word,
    input  wire [15:0] kc_cfg_id,
    input  wire [3:0]  kc_cfg_rnd_word,

    // C75 shared RAM port B and control
    input  wire        c75sh_en,
    input  wire        c75sh_we,
    input  wire [13:0] c75sh_addr,
    input  wire [15:0] c75sh_wdata,
    input  wire [1:0]  c75sh_be,
    output wire [15:0] c75sh_rdata,
    output reg         c75_run = 1'b0,
    output reg         c75_restart = 1'b0,

    // interrupts
    input  wire        vbl_event,
    input  wire        pos_event,
    input  wire [8:0]  pos_line,
    output wire [2:0]  ipl,
    output wire [3:0]  vbl_level,
    output wire [3:0]  pos_level,

    // EEPROM persistence port
    input  wire        nv_en,
    input  wire        nv_we,
    input  wire [10:1] nv_addr,
    input  wire [15:0] nv_wdata,
    output wire [15:0] nv_rdata,
    output wire        eep_cpu_write,

    // video read side
    input  wire [14:0] vid_vram_addr,
    output wire [15:0] vid_vram_data,
    input  wire [12:0] vid_pen,
    output wire [23:0] vid_rgb,
    output wire [127:0] c116_regs,
    output wire [511:0] c123_ctl,
    input  wire [15:0] roz_vram_addr,
    output wire [15:0] roz_vram_data,
    output wire [255:0] roz_ctl,            // {w15, ..., w0}
    output wire [63:0]  spr_bank,           // bytes $900008-$90000F, byte 0 ($900008) in [63:56]
    output wire [127:0] tile_bank,          // bytes $940000-$94000F, byte 0 in [127:120]
    output wire [127:0] roz_bank,           // bytes $980000-$98000F, byte 0 in [127:120]
    input  wire [14:0] obj_snap_addr,
    output wire [15:0] obj_snap_data,
    output wire [63:0] obj_pos,
    input  wire        obj_hold,

    // diagnostics
    output reg  [15:0] cls_seen = '0,
    output reg  [31:0] unmap_reads = '0,
    output reg  [31:0] unmap_writes = '0,
    output reg  [23:0] last_unmap = '0,
    output reg  [31:0] wram_sdram_cycles = '0,
    output reg  [31:0] line_hits = '0,
    output reg  [31:0] line_misses = '0,
    output reg         fill_error = 1'b0
);
    import nb2_mem_pkg::*;
    import nb2_cpu_map_pkg::*;

    localparam int LINES = 1 << LINES_LOG2;
    localparam int SW = $clog2(SLOTS);

    // ------------------------------------------------------------ hot-page slot table
    // slot_of[p] = number of hot pages below p; built in 64 clocks whenever hot_pages changes (it only changes
    // at a board-record download, with the CPU in reset).
    reg  [SW-1:0] slot_of [0:63];
    reg  [63:0]   slot_ok = '0;
    reg  [56:0]   hp_q = '0;
    reg  [6:0]    bld = 7'd64;
    reg  [6:0]    bld_cnt = '0;
    always @(posedge clk_sys) begin
        if (hot_pages != hp_q) begin
            hp_q <= hot_pages; bld <= '0; bld_cnt <= '0; slot_ok <= '0;
        end else if (!bld[6]) begin
            slot_of[bld[5:0]] <= bld_cnt[SW-1:0];
            slot_ok[bld[5:0]] <= (bld < 7'd57) && hp_q[bld[5:0]] && (bld_cnt < SLOTS);
            if ((bld < 7'd57) && hp_q[bld[5:0]]) bld_cnt <= bld_cnt + 7'd1;
            bld <= bld + 7'd1;
        end
    end

    // ------------------------------------------------------------ decode (predecoded on pd_sample)
    wire [23:0] byte_addr = {bc_addr, 1'b0};
    // predecode registers (kcap_*: loaded on pd_sample, NB2.sdc multicycle as NB-1 M22)
    cls_t       kcap_cls = CLS_UNMAP;
    reg  [5:0]  kcap_page = 6'd63;
    reg         kcap_hot = 1'b0;
    reg  [SW-1:0] kcap_slot = '0;
    cls_t       cls;
    assign cls = kcap_cls;
    reg         kcap_hit = 1'b0;
    wire [23:0] pd_byte = {pd_addr, 1'b0};
    // ROM space: rom address {data?, A19..A1}
    wire [20:1] rom_a  = {bc_addr[22], bc_addr[19:1]};
    wire [20:1] pd_rom = {pd_addr[22], pd_addr[19:1]};
    wire [LINES_LOG2 > 0 ? LINES_LOG2-1 : 0:0] idx, pd_idx;
    generate if (LINES_LOG2 > 0) begin : g_idx
        assign idx = rom_a[3 +: LINES_LOG2];
        assign pd_idx = pd_rom[3 +: LINES_LOG2];
    end else begin : g_idx0
        assign idx = 1'b0;
        assign pd_idx = 1'b0;
    end endgenerate
    (* ramstyle = "logic" *) reg [63:0] line_data [0:LINES-1];
    reg [20:3]      line_tag [0:LINES-1];
    reg [LINES-1:0] line_valid = '0;
    always @(posedge clk_sys) if (pd_sample) begin
        automatic logic [5:0] pg = wram_page(pd_byte);
        kcap_cls  <= classify(pd_byte);
        kcap_page <= pg;
        kcap_hot  <= slot_ok[pg];
        kcap_slot <= slot_of[pg];
        kcap_hit <= line_valid[pd_idx] && (line_tag[pd_idx] == pd_rom[20:3]);
    end

    wire is_write = (bc_type == 2'b11);
    wire is_iack  = (bc_fc == 3'b111) && (bc_type == 2'b10);
    wire is_rom   = (cls == CLS_ROM);
    wire is_wram  = (cls == CLS_WRAM);
    wire w_bram   = is_wram && kcap_hot;
    wire w_sdram  = is_wram && !kcap_hot;
    wire is_share = (cls == CLS_SHARE);
    wire is_cpureg = (cls == CLS_CPUREG);
    wire is_unmap = (cls == CLS_UNMAP);
    // cpureg byte writes (even bytes = uds)
    wire [3:0] cr_w = bc_addr[4:1];
    wire wr_vbl_level = is_cpureg && is_write && bc_uds && (cr_w == 4'h0);
    wire wr_pos_level = is_cpureg && is_write && bc_uds && (cr_w == 4'h1);
    wire wr_vbl_ack   = is_cpureg && is_write && bc_uds && (cr_w == 4'h2);
    wire wr_pos_ack   = is_cpureg && is_write && bc_uds && (cr_w == 4'h3);
    wire wr_c75ctl    = is_cpureg && is_write && bc_uds && (cr_w == 4'hB);
    // device reads registered one clock (SYNC_REG behaviour of NB-1 M22)
    wire is_sync = w_bram || is_share || (cls == CLS_RNG) || (cls == CLS_EEPROM) || (cls == CLS_OBJRAM) ||
                   (cls == CLS_OBJPOS) || (cls == CLS_REGS) || (cls == CLS_VRAM) || (cls == CLS_TMCTL) ||
                   (cls == CLS_ROZRAM) || (cls == CLS_ROZCTL) || (cls == CLS_C116) || (cls == CLS_KEYCUS);

    wire [1:0]  be = {bc_uds, bc_lds};

    // ------------------------------------------------------------ RNG ($1E4000, MAME machine().rand())
    reg [31:0] rng = 32'h1234_5678;
    reg [15:0] q_rng = '0;
    always @(posedge clk_sys) begin
        rng <= {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
        q_rng <= bc_addr[1] ? rng[15:0] : rng[31:16];
    end

    // ------------------------------------------------------------ RAMs and devices
    wire dev_go = bc_start && !is_iack;
    wire [15:0] q_wram, q_share, q_c116, q_c123, q_tmctl, q_rozram, q_rozctl, q_obj, q_oreg, q_eep, q_kc;

    nb1_cpu_ram #(.WORDS(SLOTS * 2048), .AW(SW + 11)) ram_work (
        .clk_sys(clk_sys), .en(dev_go && w_bram), .we(is_write),
        .addr({kcap_slot, bc_addr[11:1]}), .wdata(bc_wdata), .be(be), .rdata(q_wram));

    wire sh_wait, sh_go;
    nb1_share_lock #(.AW(14)) share_lock (
        .clk_sys(clk_sys), .reset(reset), .c_en(c75sh_en), .c_we(c75sh_we), .c_addr(c75sh_addr),
        .m_wr(dev_go && is_share && is_write), .m_addr(bc_addr[14:1]),
        .m_wait(sh_wait), .m_go(sh_go), .holds());
    nb1_dp_ram #(.WORDS(16384), .AW(14)) ram_share (
        .clk_sys(clk_sys),
        .en_a((dev_go && is_share && !sh_wait) || sh_go), .we_a(is_write), .addr_a(bc_addr[14:1]),
        .wdata_a(bc_wdata), .be_a(be), .rdata_a(q_share),
        .en_b(c75sh_en), .we_b(c75sh_we), .addr_b(c75sh_addr),
        .wdata_b(c75sh_wdata), .be_b(c75sh_be), .rdata_b(c75sh_rdata));

    nb1_c116_palette c116 (
        .clk_sys(clk_sys), .reset(reset), .access(dev_go && cls == CLS_C116),
        .write(is_write), .addr(bc_addr[14:1]), .wdata(bc_wdata), .be(be), .rdata(q_c116),
        .vid_pen(vid_pen), .vid_rgb(vid_rgb), .regs_out(c116_regs));

    nb2_c123_vram c123 (
        .clk_sys(clk_sys), .access(dev_go && cls == CLS_VRAM),
        .write(is_write), .addr(bc_addr[15:1]), .wdata(bc_wdata), .be(be), .rdata(q_c123),
        .vid_addr(vid_vram_addr), .vid_data(vid_vram_data));

    nb1_c123_ctl c123ctl (
        .clk_sys(clk_sys), .access(dev_go && cls == CLS_TMCTL),
        .write(is_write), .addr(bc_addr[5:1]), .wdata(bc_wdata), .be(be), .rdata(q_tmctl),
        .regs_out(c123_ctl), .writes());

    // C169 ROZ VRAM (64K words; port B = renderer, read only) and control (16 words)
    nb1_dp_ram #(.WORDS(65536), .AW(16)) roz_ram (
        .clk_sys(clk_sys),
        .en_a(dev_go && cls == CLS_ROZRAM), .we_a(is_write), .addr_a(bc_addr[16:1]),
        .wdata_a(bc_wdata), .be_a(be), .rdata_a(q_rozram),
        .en_b(1'b1), .we_b(1'b0), .addr_b(roz_vram_addr), .wdata_b(16'h0000), .be_b(2'b00), .rdata_b(roz_vram_data));
    wire [511:0] roz_ctl32;
    nb1_c123_ctl rozctl (
        .clk_sys(clk_sys), .access(dev_go && cls == CLS_ROZCTL),
        .write(is_write), .addr({1'b0, bc_addr[4:1]}), .wdata(bc_wdata), .be(be), .rdata(q_rozctl),
        .regs_out(roz_ctl32), .writes());
    assign roz_ctl = roz_ctl32[255:0];

    // C355: sprite RAM and position (NB-1 module; its NB-1 bank registers are unused here)
    reg  obj_pend = 1'b0;
    wire is_obj   = (cls == CLS_OBJRAM);
    wire obj_wait = bc_start && is_obj && is_write && obj_hold;
    wire obj_go   = obj_pend && !obj_hold;
    nb2_c355_ram c355 (
        .clk_sys(clk_sys),
        .obj_access((bc_start && is_obj && !obj_wait) || obj_go), .write(is_write),
        .addr(bc_addr[16:1]), .wdata(bc_wdata), .be(be), .obj_rdata(q_obj),
        .pos_access(dev_go && cls == CLS_OBJPOS), .bank_access(1'b0), .reg_addr(bc_addr[3:1]),
        .reg_rdata(q_oreg),
        .snap_addr(obj_snap_addr), .snap_data(obj_snap_data), .pos_out(obj_pos), .bank_out(),
        .unbacked_reads());
    always @(posedge clk_sys) begin
        if (reset) obj_pend <= 1'b0;
        else begin
            if (obj_wait) obj_pend <= 1'b1;
            if (obj_go) obj_pend <= 1'b0;
        end
    end

    nb1_eeprom eeprom (
        .clk_sys(clk_sys), .access(dev_go && cls == CLS_EEPROM),
        .write(is_write), .addr(bc_addr[10:1]), .wdata(bc_wdata), .be(be), .rdata(q_eep),
        .cpu_write(eep_cpu_write),
        .nv_en(nv_en), .nv_we(nv_we), .nv_addr(nv_addr), .nv_wdata(nv_wdata), .nv_rdata(nv_rdata));

    nb1_keycus keycus (
        .clk_sys(clk_sys), .reset(reset), .access(dev_go && cls == CLS_KEYCUS),
        .write(is_write), .addr(bc_addr[4:1]),
        .cfg_valid(kc_cfg_valid), .cfg_mode(kc_cfg_mode), .cfg_id_word(kc_cfg_id_word),
        .cfg_id(kc_cfg_id), .cfg_rnd_word(kc_cfg_rnd_word), .rdata(q_kc), .lfsr());

    // REGS: 8 words $640000, 4 words $900008, 8 words $940000, 8 words $980000 (power-up 0)
    reg [15:0] rg_xy [0:7];
    reg [15:0] rg_sb [0:3];
    reg [15:0] rg_tb [0:7];
    reg [15:0] rg_rb [0:7];
    integer n;
    initial for (n = 0; n < 8; n++) begin rg_xy[n] = '0; rg_tb[n] = '0; rg_rb[n] = '0; if (n < 4) rg_sb[n] = '0; end
    wire [1:0] rg_sel = (bc_addr[23:20] == 4'h6) ? 2'd0 : (bc_addr[19:18] == 2'b00) ? 2'd1 :
                        (bc_addr[19:18] == 2'b01) ? 2'd2 : 2'd3;    // $64 / $900008 / $94 / $98
    reg [15:0] q_regs = '0;
    always @(posedge clk_sys) if (dev_go && cls == CLS_REGS) begin
        case (rg_sel)
            2'd0: begin if (is_write) begin if (be[1]) rg_xy[bc_addr[3:1]][15:8] <= bc_wdata[15:8];
                                            if (be[0]) rg_xy[bc_addr[3:1]][7:0]  <= bc_wdata[7:0]; end
                        q_regs <= rg_xy[bc_addr[3:1]]; end
            2'd1: begin if (is_write) begin if (be[1]) rg_sb[bc_addr[2:1]][15:8] <= bc_wdata[15:8];
                                            if (be[0]) rg_sb[bc_addr[2:1]][7:0]  <= bc_wdata[7:0]; end
                        q_regs <= rg_sb[bc_addr[2:1]]; end
            2'd2: begin if (is_write) begin if (be[1]) rg_tb[bc_addr[3:1]][15:8] <= bc_wdata[15:8];
                                            if (be[0]) rg_tb[bc_addr[3:1]][7:0]  <= bc_wdata[7:0]; end
                        q_regs <= rg_tb[bc_addr[3:1]]; end
            default: begin if (is_write) begin if (be[1]) rg_rb[bc_addr[3:1]][15:8] <= bc_wdata[15:8];
                                               if (be[0]) rg_rb[bc_addr[3:1]][7:0]  <= bc_wdata[7:0]; end
                        q_regs <= rg_rb[bc_addr[3:1]]; end
        endcase
    end
    assign spr_bank  = {rg_sb[0], rg_sb[1], rg_sb[2], rg_sb[3]};
    assign tile_bank = {rg_tb[0], rg_tb[1], rg_tb[2], rg_tb[3], rg_tb[4], rg_tb[5], rg_tb[6], rg_tb[7]};
    assign roz_bank  = {rg_rb[0], rg_rb[1], rg_rb[2], rg_rb[3], rg_rb[4], rg_rb[5], rg_rb[6], rg_rb[7]};

    // ------------------------------------------------------------ ROM: L0 line buffer + L1 cache
    reg         pc_owns = 1'b0;           // the memory port's outstanding request is the cache's
    wire [18:0] w_off = wram_offset(byte_addr);
    wire        pc_done, pc_err, pc_hit, pc_miss, pc_mreq_valid;
    wire [15:0] pc_rdata;
    wire [63:0] pc_line;
    wire [3:0]  pc_mreq_region;
    wire [25:0] pc_mreq_offset;
    wire        hit = kcap_hit;
    reg  pc_req_q = 1'b0;
    always @(posedge clk_sys) pc_req_q <= !reset && bc_start && is_rom && !is_write && !hit;
    nb2_rom_cache #(.LINES_LOG2(PCACHE_LOG2)) rcache (
        .clk_sys(clk_sys), .reset(reset), .bypass(pcache_bypass),
        .req(pc_req_q), .addr(rom_a),
        .done(pc_done), .rdata(pc_rdata), .line(pc_line), .err(pc_err), .hit_pulse(pc_hit), .miss_pulse(pc_miss),
        .mreq_valid(pc_mreq_valid), .mreq_ready(mreq_ready && pc_mreq_valid), .mreq_region(pc_mreq_region),
        .mreq_offset(pc_mreq_offset), .mrsp_valid(mrsp_valid && pc_owns), .mrsp_line(mrsp_line), .mrsp_err(mrsp_err));
    wire [63:0] ld = line_data[idx];
    reg  [15:0] rom_word;
    always_comb case (bc_addr[2:1])
        2'd0: rom_word = ld[63:48];
        2'd1: rom_word = ld[47:32];
        2'd2: rom_word = ld[31:16];
        default: rom_word = ld[15:0];
    endcase

    // ------------------------------------------------------------ SDRAM work RAM
    reg         ws_valid = 1'b0;          // request posted
    reg         ws_busy = 1'b0;           // the current bus cycle waits for its SDRAM access
    reg         ws_we = 1'b0;
    reg  [25:0] ws_off = '0;
    reg  [31:0] ws_wd = '0;
    reg  [3:0]  ws_be = '0;
    reg         ws_lo = 1'b0;
    always @(posedge clk_sys) begin
        if (pc_mreq_valid && mreq_ready) pc_owns <= 1'b1;
        else if (mrsp_valid) pc_owns <= 1'b0;
        if (reset) pc_owns <= 1'b0;
    end
    assign mreq_valid  = pc_mreq_valid || ws_valid;
    assign mreq_region = pc_mreq_valid ? pc_mreq_region : REG_WRAM;
    assign mreq_offset = pc_mreq_valid ? pc_mreq_offset : ws_off;
    assign mreq_we     = !pc_mreq_valid && ws_we;
    assign mreq_wdata  = ws_wd;
    assign mreq_be     = pc_mreq_valid ? 4'hF : ws_be;

    // ------------------------------------------------------------ response
    wire done_now = is_iack || (is_write && !obj_wait && !sh_wait && !w_sdram) || (is_rom && !is_write && hit) ||
                    (is_rom && is_write) || is_unmap || (is_cpureg);
    reg  done_q = 1'b0;
    reg  filling = 1'b0;
    reg  err_q = 1'b0;
    reg  rd1 = 1'b0;
    reg  [15:0] rd_q = '0;
    assign bc_done = bc_start ? done_now : done_q;

    reg  [15:0] dev_rdata;
    always_comb case (cls)
        CLS_WRAM:   dev_rdata = q_wram;
        CLS_SHARE:  dev_rdata = q_share;
        CLS_RNG:    dev_rdata = q_rng;
        CLS_EEPROM: dev_rdata = q_eep;
        CLS_OBJRAM: dev_rdata = q_obj;
        CLS_OBJPOS: dev_rdata = q_oreg;
        CLS_REGS:   dev_rdata = q_regs;
        CLS_VRAM:   dev_rdata = q_c123;
        CLS_TMCTL:  dev_rdata = q_tmctl;
        CLS_ROZRAM: dev_rdata = q_rozram;
        CLS_ROZCTL: dev_rdata = q_rozctl;
        CLS_C116:   dev_rdata = q_c116;
        CLS_KEYCUS: dev_rdata = q_kc;
        default:    dev_rdata = 16'h0000;
    endcase
    always_comb begin
        if (is_write || is_iack || err_q) bc_rdata = 16'hFFFF;
        else if (is_cpureg)                bc_rdata = 16'hFFFF;
        else if (is_unmap)                 bc_rdata = 16'h0000;
        else if (is_rom)                   bc_rdata = rom_word;
        else                               bc_rdata = rd_q;      // registered device / SDRAM work-RAM data
    end

    always @(posedge clk_sys) begin
        if (reset) begin
            done_q <= 1'b0; rd1 <= 1'b0; filling <= 1'b0; err_q <= 1'b0;
            line_valid <= '0; fill_error <= 1'b0; ws_valid <= 1'b0; ws_busy <= 1'b0;
        end else begin
            rd1 <= 1'b0;
            if (rd1) begin rd_q <= dev_rdata; done_q <= 1'b1; end
            if (bc_start) begin
                done_q <= done_now && !obj_wait && !sh_wait;
                rd1    <= is_sync && !is_write && !obj_wait && !sh_wait;
                err_q  <= 1'b0;
                if (is_sync && is_write) done_q <= !obj_wait && !sh_wait;
                if (is_rom && !is_write && !hit) begin
                    filling <= 1'b1;
                    done_q  <= 1'b0;
                end
                if (w_sdram && !is_iack) begin
                    // one SDRAM access for this bus cycle (16-bit word in its longword lane)
                    ws_valid <= 1'b1;
                    ws_busy  <= 1'b1;
                    ws_we    <= is_write;
                    ws_off   <= {7'd0, w_off[18:2], 2'b00};
                    ws_lo    <= bc_addr[1];
                    ws_wd    <= {bc_wdata, bc_wdata};
                    ws_be    <= bc_addr[1] ? {2'b00, be} : {be, 2'b00};
                    done_q   <= 1'b0;
                end
            end
            if (obj_go || sh_go) done_q <= 1'b1;
            if (ws_valid && mreq_ready && !pc_mreq_valid) ws_valid <= 1'b0;
            if (ws_busy && !ws_valid && mrsp_valid && !pc_owns) begin
                ws_busy <= 1'b0;
                done_q  <= 1'b1;
                rd_q    <= ws_lo ? mrsp_rdata[15:0] : mrsp_rdata[31:16];
            end
            if (filling && pc_done) begin
                filling <= 1'b0;
                done_q  <= 1'b1;
                if (pc_err) begin
                    err_q      <= 1'b1;
                    fill_error <= 1'b1;
                end else begin
                    line_data[idx]  <= pc_line;
                    line_tag[idx]   <= rom_a[20:3];
                    line_valid[idx] <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------ C75 control ($F00016 bit 0)
    always @(posedge clk_sys) begin
        c75_restart <= 1'b0;
        if (reset) c75_run <= 1'b0;
        else if (bc_start && wr_c75ctl) begin
            c75_run     <= bc_wdata[8];
            c75_restart <= bc_wdata[8];
        end
    end

    // ------------------------------------------------------------ interrupts
    nb1_main_irq main_irq (
        .clk_sys(clk_sys), .reset(reset), .vbl_event(vbl_event),
        .wr_level(bc_start && wr_vbl_level), .wr_ack(bc_start && wr_vbl_ack),
        .wdata(bc_wdata[15:8]),
        .iack(bc_start && is_iack), .iack_level(bc_addr[3:1]), .ipl(ipl),
        .vbl_level(vbl_level), .vbl_pending(), .vbl_events(), .vbl_raised(), .vbl_taken(), .vbl_acks(),
        .level_writes(),
        .pos_event(pos_event), .pos_line(pos_line), .pos_reg5(c116_regs[5*16 +: 16]),
        .wr_pos_level(bc_start && wr_pos_level), .wr_pos_ack(bc_start && wr_pos_ack),
        .pos_wdata(bc_wdata[15:8]), .pos_level(pos_level), .pos_pending(), .pos_raised(), .pos_taken(),
        .pos_acks(), .pos_frame(), .pos_ack_frame(), .pos_odd_frames());

    // ------------------------------------------------------------ statistics
    always @(posedge clk_sys) begin
        if (reset) begin
            cls_seen <= '0; unmap_reads <= '0; unmap_writes <= '0; wram_sdram_cycles <= '0;
            line_hits <= '0; line_misses <= '0;
        end else begin
            if (bc_start && !is_iack) begin
                cls_seen[cls] <= 1'b1;
                if (is_unmap) begin
                    last_unmap <= byte_addr;
                    if (is_write) unmap_writes <= unmap_writes + 32'd1;
                    else          unmap_reads  <= unmap_reads + 32'd1;
                end
                if (w_sdram) wram_sdram_cycles <= wram_sdram_cycles + 32'd1;
                if (is_rom && !is_write && hit) line_hits <= line_hits + 32'd1;
            end
            if (pc_hit)  line_hits   <= line_hits + 32'd1;
            if (pc_miss) line_misses <= line_misses + 32'd1;
        end
    end
endmodule
