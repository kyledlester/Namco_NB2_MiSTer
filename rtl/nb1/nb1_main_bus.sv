// Namco NB-1 MiSTer core -- 68EC020 main bus (M3).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Services the bus cycles captured by nb1_cpu (bc_*), one response per
// cycle, using the NB-1 address map in nb1_cpu_map_pkg:
//
//   ROM   $000000-$0FFFFF  region PROG through the program line buffer below
//   RAM   $1C0000-$1C3FFF, $200000-$207FFF, $208000-$23FFFF   BRAM
//   C116 $700000-$707FFF    M4 CPU-visible palette RAM/register storage
//   C123 $640000-$64FFFF    M5 CPU-visible tile VRAM (32K x 16, no mirrors)
//   C123 ctl $660000-$66003F M10 control registers (32 words, read/write,
//                          nb1_c123_ctl); VRAM, control and the C116 palette
//                          also have M10 video read ports for the renderer
//   C355 $600000-$61FFFF, $620000-$620007, $680000-$68000F  M11 sprite RAM,
//                          position and tile bank (nb1_c355_ram). A sprite-RAM
//                          write that arrives while the renderer copies its
//                          vblank snapshot (obj_hold) waits until it ends.
//   EEPROM $580000-$5807FF  M6 CPU-visible 2816 (2 KiB contiguous bytes, all lanes);
//                          M18: second storage port for MiSTer NVRAM (nv_*)
//   KEYCUS $6E0000-$6E001F  M7 generic C3xx (16 words; ID/changing word from
//                          the MRA board record; writes ignored)
//   C75 control $400018     M8: byte write, bit 0 = 1 releases the C75 (and
//                          restarts it: MAME clears HALT and pulses RESET on
//                          every such write), 0 halts it.
//   VBL IRQ $400004/$400009 M9: byte writes of the VBL level ($400004, D15-8)
//                          and the VBL ack ($400009, D7-0) go to nb1_main_irq.
//   POS IRQ $400001/$400006 M13: byte writes of the POS level ($400001, D7-0)
//                          and the POS ack ($400006, D15-8) go to nb1_main_irq.
//                          The rest of cpureg $400000-$40001F (unknown level
//                          and ack, watchdog, ...) stays unimplemented;
//                          every cpureg read returns $FF like MAME.
//   IACK (FC = 111)        M9: the TG68K interrupt-acknowledge read cycle is
//                          answered at once ($FFFF, ignored: autovectors) and is
//                          not a program-space access (no class, not counted
//                          as unimplemented).
//   shared RAM $200000-$207FFF is now dual-port (M8): port B belongs to the
//                          C75 (its $4000-$BFFF), see nb1_c75.sv.
//   other classes          UNIMPLEMENTED: reads return $FFFF, writes
//                          are dropped; every such access is counted and the
//                          first/last ones are latched for the overlay.
//   ROM writes             dropped and counted (the EPROMs ignore them).
//
// Response contract with nb1_cpu: bc_start is a one-cycle pulse per bus
// cycle; bc_done must be valid combinationally in that same cycle for
// accesses that need no wait (writes, unimplemented, line-buffer hits) and
// then stays high (with bc_rdata stable) until the next bc_start. Each bus
// cycle is acted on only in its bc_start cycle or by the one miss/RAM-read
// sequence it starts, so it can never be performed twice however long the
// CPU then waits for a scheduler credit.
//
// Program line buffer [IMPLEMENTATION-DECISION, docs/M3_RESEARCH.md s.6]:
// LINES (power of two) direct-mapped lines of 8 bytes, the SDRAM
// controller's native 4-word burst (nb1_memory rsp_line). A miss costs one
// ordinary SDRAM read (~15 clk_sys); the following fetches from the same
// line are free. PROG is read-only while the CPU runs (downloads hold the
// CPU in reset; CPU writes to ROM never reach SDRAM), so a line can never
// be stale: no invalidation on writes is needed. Branches simply miss or
// hit by tag. The buffer is emptied whenever the CPU is reset.
//
// M12 program cache [docs/M12_RESEARCH.md, docs/M12_PLAN.md]: PCACHE_LOG2 > 0
// puts nb1_prog_cache (2**PCACHE_LOG2 direct-mapped 8-byte lines in M10K) as
// an L1 behind the unchanged buffer above (L0): L0 hits still complete in the
// bc_start cycle (so the SR-2 rate of tight loops is untouched); an L0 miss
// looks up the L1 (~4 clk on an L1 hit) instead of going to SDRAM, and only an
// L1 miss reads SDRAM. Only 68020 reads of CLS_ROM (instruction fetches and
// ROM data reads) use it; RAMs and devices are unchanged. pcache_bypass = 1:
// the L1 passes every L0 miss straight to SDRAM (M3-M11 behaviour, one clock
// later per miss) for A/B tests. Statistics: line_hits = L0 + L1 hits,
// line_misses = SDRAM program reads (as in M3-M11).
// PCACHE_LOG2 = 0 (default, every pre-M12 bench) = the M3-M11 buffer alone.
module nb1_main_bus #(parameter int LINES_LOG2 = 0, parameter int PCACHE_LOG2 = 0,
                      parameter bit PREDECODE = 1'b0,     // M22: 1 = decode from nb1_cpu's pd_* (production)
                      parameter bit SYNC_REG  = 1'b1) (   // M22: device read data registered (+1 clk, see below)
    input  wire        clk_sys,
    input  wire        reset,           // = CPU reset: flush and clear statistics
    // M22 predecode (PREDECODE = 1): nb1_cpu's kcap load strobe and the kernel address it loads; the
    // class, the work-RAM word index and the program line-buffer hit of the NEXT bus cycle are
    // registered here on the same edge as kcap_addr (kernel -> kcap_* multicycle, NB1.sdc), so the
    // bc_start cycle uses registers instead of the address decoder (R-TIMING-1, docs/M22_TIMING_CLOSURE.md).
    input  wire        pd_sample,
    input  wire [23:1] pd_addr,
    input  wire        pcache_bypass,   // M12: program cache in 2-line geometry (PCACHE_LOG2 > 0 only)

    // from nb1_cpu
    input  wire        bc_start,
    input  wire [23:1] bc_addr,
    input  wire [1:0]  bc_type,         // 00 fetch, 10 read, 11 write
    input  wire [2:0]  bc_fc,           // M9: function code, 111 = interrupt acknowledge
    input  wire        bc_uds,
    input  wire        bc_lds,
    input  wire [15:0] bc_wdata,
    output wire        bc_done,
    output reg  [15:0] bc_rdata,

    // nb1_memory client port (region PROG line fills only)
    output wire        mreq_valid,
    input  wire        mreq_ready,
    output wire [3:0]  mreq_region,
    output wire [24:0] mreq_offset,
    output wire        mreq_we,
    output wire [31:0] mreq_wdata,
    output wire [3:0]  mreq_be,
    input  wire        mrsp_valid,
    input  wire [63:0] mrsp_line,
    input  wire        mrsp_err,

    // M8 C75: shared RAM port B and the $400018 control output
    input  wire        c75sh_en,
    input  wire        c75sh_we,
    input  wire [13:0] c75sh_addr,
    input  wire [15:0] c75sh_wdata,
    input  wire [1:0]  c75sh_be,
    output wire [15:0] c75sh_rdata,
    // M25: misc read ports -- the gun I/O board counts {X p1, Y p1, X p2, Y p2} (nb1_gun; 0 without one)
    input  wire [31:0] gun_counts,
    output reg         c75_run = 1'b0,      // $400018 bit 0 (1 = released)
    output reg         c75_restart = 1'b0,  // one-cycle pulse on every write of 1
    output reg  [7:0]  c75_ctl_writes = '0, // $400018 writes since CPU reset

    // M9 main-CPU VBL interrupt (nb1_main_irq)
    input  wire        vbl_event,           // start of line 224 (nb1_video_timing vblank_begin)
    output wire [2:0]  ipl,                 // to nb1_cpu
    output wire [3:0]  vbl_level,
    output wire        vbl_pending,
    output wire [31:0] vbl_events,
    output wire [31:0] vbl_raised,
    output wire [31:0] vbl_taken,
    output wire [31:0] vbl_acks,
    output reg  [31:0] iack_cycles = '0,    // every interrupt-acknowledge cycle
    output reg  [2:0]  last_iack_level = '0,
    // M13 POS interrupt (nb1_main_irq; CPU raster from nb1_cpu_raster)
    input  wire        pos_event,           // start of every CPU line
    input  wire [8:0]  pos_line,            // the CPU line starting
    output wire [3:0]  pos_level,
    output wire        pos_pending,
    output wire [31:0] pos_raised,
    output wire [31:0] pos_taken,
    output wire [31:0] pos_acks,
    output wire [8:0]  pos_frame,           // POS requests in the last CPU frame
    output wire [8:0]  pos_ack_frame,       // POS acks in the last CPU frame
    output wire [15:0] pos_odd_frames,      // frames with a POS count other than 0/224 or acks != count
    // first unimplemented accesses after the first accepted VBL interrupt;
    // the write latch skips cpureg (watchdog, POS level/ack, ... are already
    // known and written every frame by the VBL handler)
    output reg         pv_unr_valid = 1'b0,
    output reg  [23:0] pv_unr_addr = '0,
    output reg         pv_unw_valid = 1'b0,
    output reg  [23:0] pv_unw_addr = '0,
    output reg  [15:0] pv_unw_data = '0,
    output reg  [31:0] pv_unr_count = '0,   // unimplemented reads after the first VBL

    // M18 EEPROM persistence port (nb1_nvram; independent of the CPU port)
    input  wire        nv_en,
    input  wire        nv_we,
    input  wire [10:1] nv_addr,
    input  wire [15:0] nv_wdata,
    output wire [15:0] nv_rdata,
    output wire        eep_cpu_write,      // one pulse per committed CPU EEPROM write
    // M7 KEYCUS board description (nb1_board_config, MRA ioctl index 2)
    input  wire        kc_cfg_valid,
    input  wire [7:0]  kc_cfg_mode,
    input  wire [3:0]  kc_cfg_id_word,
    input  wire [15:0] kc_cfg_id,
    input  wire [3:0]  kc_cfg_rnd_word,

    // M10 video read side (nb1_c123_render / nb1_video_out)
    input  wire [14:0] vid_vram_addr,
    output wire [15:0] vid_vram_data,
    input  wire [12:0] vid_pen,
    output wire [23:0] vid_rgb,
    output wire [127:0] c116_regs,
    output wire [511:0] c123_ctl,
    output wire [31:0] c123_ctl_writes,

    // M11 C355 sprite side (nb1_c355_render)
    input  wire [14:0] obj_snap_addr,
    output wire [15:0] obj_snap_data,
    output wire [63:0] obj_pos,
    output wire [127:0] obj_bank,
    input  wire        obj_hold,
    output wire [31:0] obj_unbacked_reads,
    output reg  [31:0] obj_writes = '0,
    output reg  [15:0] obj_holds = '0,      // CPU sprite-RAM writes that had to wait

    // debug / status
    output reg  [15:0] cls_seen = '0,        // bit per nb1_cpu_map_pkg class touched
    output reg  [31:0] unimpl_reads = '0,
    output reg  [31:0] unimpl_writes = '0,
    output reg  [23:0] first_unimpl_addr = '0,
    output reg         first_unimpl_we = 1'b0,
    output reg         first_unimpl_valid = 1'b0,
    output reg  [23:0] last_unimpl_raddr = '0,
    output reg  [23:0] last_unimpl_waddr = '0,
    output reg  [15:0] last_unimpl_wdata = '0,
    output reg  [31:0] rom_writes = '0,
    output reg  [31:0] ram_reads = '0,
    output reg  [31:0] ram_writes = '0,
    output reg  [31:0] c116_reads = '0,
    output reg  [31:0] c116_writes = '0,
    output reg  [31:0] c123_reads = '0,
    output reg  [31:0] c123_writes = '0,
    output reg  [23:0] last_c123_addr = '0,  // byte address of the last C123 VRAM cycle
    output reg         last_c123_we = 1'b0,
    output reg  [31:0] eep_reads = '0,
    output reg  [31:0] eep_writes = '0,
    output reg  [23:0] last_eep_addr = '0,   // byte address of the last EEPROM cycle
    output reg         last_eep_we = 1'b0,
    output reg  [31:0] kc_reads = '0,
    output reg  [31:0] kc_writes = '0,
    output reg  [23:0] last_kc_addr = '0,    // byte address of the last KEYCUS cycle
    output reg         last_kc_we = 1'b0,
    output reg  [15:0] last_kc_data = '0,    // word read or written by that cycle
    output reg  [31:0] line_hits = '0,
    output reg  [31:0] line_misses = '0,
    output reg         fill_error = 1'b0      // an SDRAM line fill returned rsp_err
);
    import nb1_mem_pkg::*;
    import nb1_cpu_map_pkg::*;

    localparam int LINES = 1 << LINES_LOG2;

    assign mreq_region = REG_PROG;
    assign mreq_we     = 1'b0;
    assign mreq_wdata  = '0;
    assign mreq_be     = 4'b1111;

    // ------------------------------------------------------------ decode
    wire [23:0] byte_addr = {bc_addr, 1'b0};
    cls_t       cls;
    // M22: the registered predecode (see the port comment). Same values as the combinational decode of
    // bc_addr in every bus cycle: bc_addr IS the address loaded on pd_sample (nb1_cpu kcap_addr).
    cls_t       kcap_cls = CLS_UNMAP;
    reg  [16:0] kcap_wram = '0;
    reg         kcap_hit = 1'b0;
    always_comb cls = PREDECODE ? kcap_cls : classify(byte_addr);
    wire        is_write = (bc_type == 2'b11);
    // M8: the C75 control byte is the even byte of cpureg word $400018.
    // Declare it before is_impl for ModelSim 10.5b (no implicit forward net).
    wire        is_c75ctl = (cls == CLS_CPUREG) && (bc_addr[4:1] == 4'hC);
    // M9: byte writes $400004 (VBL level, even byte) and $400009 (VBL ack, odd byte)
    wire        wr_vbl_level = (cls == CLS_CPUREG) && is_write && (bc_addr[4:1] == 4'h2) && bc_uds;
    wire        wr_vbl_ack   = (cls == CLS_CPUREG) && is_write && (bc_addr[4:1] == 4'h4) && bc_lds;
    // M13: byte writes $400001 (POS level, odd byte) and $400006 (POS ack, even byte)
    wire        wr_pos_level = (cls == CLS_CPUREG) && is_write && (bc_addr[4:1] == 4'h0) && bc_lds;
    wire        wr_pos_ack   = (cls == CLS_CPUREG) && is_write && (bc_addr[4:1] == 4'h3) && bc_uds;
    // M9: TG68K interrupt acknowledge (CPU space, not the NB-1 address map)
    wire        is_iack  = (bc_fc == 3'b111) && (bc_type == 2'b10);
    wire        is_impl  = implemented(cls) || is_c75ctl || wr_vbl_level || wr_vbl_ack || wr_pos_level || wr_pos_ack || is_iack;
    // M13 (timing): the response path (bc_rdata / done_now -> TG68K, R-TIMING-1 cone) uses a predicate
    // without the write-only cpureg decodes. Identical behaviour: a write answers $FFFF and completes
    // at once whether or not it is implemented; for a read the write decodes are 0.
    wire        rsp_impl = implemented(cls) || is_c75ctl || is_iack;
    wire        is_ram   = (cls == CLS_RAM1C) || (cls == CLS_SHARE) || (cls == CLS_WRAM);
    wire        is_c116  = (cls == CLS_C116);
    wire        is_c123  = (cls == CLS_VRAM);
    wire        is_tmctl = (cls == CLS_TMCTL);
    wire        is_obj   = (cls == CLS_OBJRAM);
    wire        is_opos  = (cls == CLS_OBJPOS);
    wire        is_obank = (cls == CLS_SPRBANK);
    wire        is_eep   = (cls == CLS_EEPROM);
    wire        is_kc    = (cls == CLS_KEYCUS);
    wire        is_misc  = (cls == CLS_RNG);
    wire        is_sync  = is_ram || is_c116 || is_c123 || is_tmctl || is_eep || is_kc || is_obj || is_opos || is_obank || is_misc;

    // M25: CLS_RNG = the random generator $1E4000-$1E4003 [MAME-SOURCE randgen_r: machine().rand(), a new
    // value per read; srand_w a no-op] -- here a free-running 32-bit LFSR, so successive reads differ --
    // and the gun I/O board $100000-$10001F [MAME-SOURCE gunbulet_state::gun_r: the byte at $100000 + 4n,
    // n = 0/1 Y p2, 2/3 X p2, 4/5 Y p1, 6/7 X p1; every other byte 0]. q_misc follows the cycle's address
    // every clock, so it is valid when rd1 samples dev_rdata (one clock after bc_start).
    reg  [31:0] rng = 32'h1234_5678;
    reg  [15:0] q_misc = 16'h0000;
    always @(posedge clk_sys) begin
        rng <= {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
        if (byte_addr[19])                       // $1E4000: random
            q_misc <= bc_addr[1] ? rng[15:0] : rng[31:16];
        else if (bc_addr[1])                     // gun: odd words are 0
            q_misc <= 16'h0000;
        else case (bc_addr[4:3])
            2'd0: q_misc <= {gun_counts[7:0],   8'h00};   // Y p2
            2'd1: q_misc <= {gun_counts[15:8],  8'h00};   // X p2
            2'd2: q_misc <= {gun_counts[23:16], 8'h00};   // Y p1
            2'd3: q_misc <= {gun_counts[31:24], 8'h00};   // X p1
        endcase
    end
    wire        is_rom   = (cls == CLS_ROM);

    // --------------------------------------------------------------- RAMs
    wire ram_en = bc_start && is_ram;
    wire ram_we = is_write;
    wire [1:0] be = {bc_uds, bc_lds};
    wire [15:0] q_1c, q_share, q_wram;

    // M25: words 0..2047 = $1C0000-$1C0FFF, words 2048..4095 = $240000-$240FFF (address bit 21 picks)
    nb1_cpu_ram #(.WORDS(RAM1C_BYTES), .AW($clog2(RAM1C_BYTES))) ram_1c (
        .clk_sys(clk_sys), .en(ram_en && cls == CLS_RAM1C), .we(ram_we),
        .addr({bc_addr[21], bc_addr[$clog2(RAM1C_BYTES / 2):1]}), .wdata(bc_wdata), .be(be), .rdata(q_1c));

    // shared RAM: port A = 68020, port B = C75 (M8)
    // M24: a 68020 write into a word the C75 is reading-modifying-writing is held until the C75's write-back
    // (nb1_share_lock: the C75 sound driver takes mailbox commands with a read-modify-write).
    wire sh_wait, sh_go;
    nb1_share_lock #(.AW(14)) share_lock (
        .clk_sys(clk_sys), .reset(reset), .c_en(c75sh_en), .c_we(c75sh_we), .c_addr(c75sh_addr),
        .m_wr(bc_start && cls == CLS_SHARE && is_write), .m_addr(bc_addr[14:1]),
        .m_wait(sh_wait), .m_go(sh_go), .holds());
    nb1_dp_ram #(.WORDS(16384), .AW(14)) ram_share (
        .clk_sys(clk_sys),
        .en_a((ram_en && cls == CLS_SHARE && !sh_wait) || sh_go), .we_a(ram_we), .addr_a(bc_addr[14:1]),
        .wdata_a(bc_wdata), .be_a(be), .rdata_a(q_share),
        .en_b(c75sh_en), .we_b(c75sh_we), .addr_b(c75sh_addr),
        .wdata_b(c75sh_wdata), .be_b(c75sh_be), .rdata_b(c75sh_rdata));

    // $208000-$23FFFF: 224 KiB = 114688 words, word index = (A - $208000) / 2
    wire [22:0] wram_word_c = bc_addr[23:1] - 23'h104000;
    wire [16:0] wram_word = PREDECODE ? kcap_wram : wram_word_c[16:0];
    nb1_cpu_ram #(.WORDS(114688), .AW(17)) ram_work (
        .clk_sys(clk_sys), .en(ram_en && cls == CLS_WRAM), .we(ram_we),
        .addr(wram_word), .wdata(bc_wdata), .be(be), .rdata(q_wram));

    wire [15:0] q_c116;
    nb1_c116_palette c116 (
        .clk_sys(clk_sys), .reset(reset), .access(bc_start && is_c116),
        .write(is_write), .addr(bc_addr[14:1]), .wdata(bc_wdata), .be(be), .rdata(q_c116),
        .vid_pen(vid_pen), .vid_rgb(vid_rgb), .regs_out(c116_regs));

    wire [15:0] q_c123;
    nb1_c123_vram c123 (
        .clk_sys(clk_sys), .access(bc_start && is_c123),
        .write(is_write), .addr(bc_addr[15:1]), .wdata(bc_wdata), .be(be), .rdata(q_c123),
        .vid_addr(vid_vram_addr), .vid_data(vid_vram_data));

    wire [15:0] q_tmctl;
    nb1_c123_ctl c123ctl (
        .clk_sys(clk_sys), .access(bc_start && is_tmctl),
        .write(is_write), .addr(bc_addr[5:1]), .wdata(bc_wdata), .be(be), .rdata(q_tmctl),
        .regs_out(c123_ctl), .writes(c123_ctl_writes));

    // M11: C355. A write held by obj_hold is performed when the hold ends.
    reg  obj_pend = 1'b0;
    wire obj_wait = bc_start && is_obj && is_write && obj_hold;
    wire obj_go   = obj_pend && !obj_hold;
    wire [15:0] q_obj, q_oreg;
    nb1_c355_ram c355 (
        .clk_sys(clk_sys),
        .obj_access((bc_start && is_obj && !obj_wait) || obj_go), .write(is_write),
        .addr(bc_addr[16:1]), .wdata(bc_wdata), .be(be), .obj_rdata(q_obj),
        .pos_access(bc_start && is_opos), .bank_access(bc_start && is_obank), .reg_addr(bc_addr[3:1]),
        .reg_rdata(q_oreg),
        .snap_addr(obj_snap_addr), .snap_data(obj_snap_data), .pos_out(obj_pos), .bank_out(obj_bank),
        .unbacked_reads(obj_unbacked_reads));
    always @(posedge clk_sys) begin
        if (reset) begin
            obj_pend   <= 1'b0;
            obj_writes <= '0;
            obj_holds  <= '0;
        end else begin
            if (obj_wait) begin
                obj_pend <= 1'b1;
                if (obj_holds != 16'hFFFF) obj_holds <= obj_holds + 16'd1;
            end
            if (obj_go) obj_pend <= 1'b0;
            if ((bc_start && is_obj && is_write && !obj_hold) || obj_go) obj_writes <= obj_writes + 32'd1;
        end
    end

    wire [15:0] q_eep;
    nb1_eeprom eeprom (
        .clk_sys(clk_sys), .access(bc_start && is_eep),
        .write(is_write), .addr(bc_addr[10:1]), .wdata(bc_wdata), .be(be), .rdata(q_eep),
        .cpu_write(eep_cpu_write),
        .nv_en(nv_en), .nv_we(nv_we), .nv_addr(nv_addr), .nv_wdata(nv_wdata), .nv_rdata(nv_rdata));

    wire [15:0] q_kc;
    nb1_keycus keycus (
        .clk_sys(clk_sys), .reset(reset), .access(bc_start && is_kc),
        .write(is_write), .addr(bc_addr[4:1]),
        .cfg_valid(kc_cfg_valid), .cfg_mode(kc_cfg_mode), .cfg_id_word(kc_cfg_id_word),
        .cfg_id(kc_cfg_id), .cfg_rnd_word(kc_cfg_rnd_word),
        .rdata(q_kc), .lfsr());

    // ------------------------------------------------- program line buffer
    (* ramstyle = "logic" *) reg [63:0]  line_data  [0:LINES-1];   // M16: 2 lines; kept out of M10K
    reg [19:3]  line_tag   [0:LINES-1];
    reg [LINES-1:0] line_valid = '0;

    wire [LINES_LOG2 > 0 ? LINES_LOG2-1 : 0:0] idx;
    generate if (LINES_LOG2 > 0) begin : g_idx
        assign idx = bc_addr[3 +: LINES_LOG2];
    end else begin : g_idx0
        assign idx = 1'b0;
    end endgenerate

    localparam bit PC = (PCACHE_LOG2 > 0);
    wire        hit_c = line_valid[idx] && (line_tag[idx] == bc_addr[19:3]);
    wire        hit = PREDECODE ? kcap_hit : hit_c;
    // M22 predecode registers. The line tags cannot change between pd_sample and the end of the bus cycle
    // it describes: a fill belongs to the current bus cycle and completes before its done, i.e. before the
    // enable edge that precedes the next pd_sample.
    wire [LINES_LOG2 > 0 ? LINES_LOG2-1 : 0:0] pd_idx;
    generate if (LINES_LOG2 > 0) begin : g_pdidx
        assign pd_idx = pd_addr[3 +: LINES_LOG2];
    end else begin : g_pdidx0
        assign pd_idx = 1'b0;
    end endgenerate
    wire [22:0] pd_wram = pd_addr[23:1] - 23'h104000;
    always @(posedge clk_sys) if (pd_sample) begin
        kcap_cls  <= classify({pd_addr, 1'b0});
        kcap_wram <= pd_wram[16:0];
        kcap_hit  <= line_valid[pd_idx] && (line_tag[pd_idx] == pd_addr[19:3]);
    end
    reg         lg_mreq_valid = 1'b0;
    reg  [24:0] lg_mreq_offset = '0;

    // M12 program cache
    wire        pc_done, pc_err, pc_hit, pc_miss, pc_mreq_valid;
    wire [15:0] pc_rdata;
    wire [63:0] pc_line;
    wire [24:0] pc_mreq_offset;
    // The L1 request is registered one clock after bc_start (bc_addr is held for
    // the whole bus cycle), so the CPU address decode and the L0 compare end in
    // a flip-flop, not in the M10K read port (keeps the R-TIMING-1 cone as is).
    reg pc_req_q = 1'b0;
    always @(posedge clk_sys) pc_req_q <= !reset && bc_start && is_rom && !is_write && !hit;
    generate if (PCACHE_LOG2 > 0) begin : g_pc
        nb1_prog_cache #(.LINES_LOG2(PCACHE_LOG2)) pcache (
            .clk_sys(clk_sys), .reset(reset), .bypass(pcache_bypass),
            .req(pc_req_q), .addr(bc_addr[19:1]),   // L0 misses only
            .done(pc_done), .rdata(pc_rdata), .line(pc_line), .err(pc_err), .hit_pulse(pc_hit), .miss_pulse(pc_miss),
            .mreq_valid(pc_mreq_valid), .mreq_ready(mreq_ready), .mreq_offset(pc_mreq_offset),
            .mrsp_valid(mrsp_valid), .mrsp_line(mrsp_line), .mrsp_err(mrsp_err));
    end else begin : g_nopc
        assign pc_done = 1'b0; assign pc_err = 1'b0; assign pc_hit = 1'b0; assign pc_miss = 1'b0;
        assign pc_mreq_valid = 1'b0; assign pc_rdata = 16'hFFFF; assign pc_mreq_offset = '0; assign pc_line = '0;
    end endgenerate
    assign mreq_valid  = PC ? pc_mreq_valid  : lg_mreq_valid;
    assign mreq_offset = PC ? pc_mreq_offset : lg_mreq_offset;
    wire [63:0] ld  = line_data[idx];
    reg  [15:0] rom_word;
    always_comb case (bc_addr[2:1])
        2'd0: rom_word = ld[63:48];
        2'd1: rom_word = ld[47:32];
        2'd2: rom_word = ld[31:16];
        default: rom_word = ld[15:0];
    endcase

    // ------------------------------------------------------ response state
    // done_now: the cycle can complete in its bc_start cycle.
    wire done_now = !rsp_impl || (is_write && !obj_wait && !sh_wait) || (is_rom && hit) || is_c75ctl || is_iack;
    reg  done_q  = 1'b0;
    reg  filling = 1'b0;       // line fill in flight for the current cycle
    reg  err_q   = 1'b0;       // current ROM read got rsp_err: answer $FFFF
    assign bc_done = bc_start ? done_now : done_q;

    // M22 (R-TIMING-1, RAM read data): with SYNC_REG a synchronous device read (CPU RAMs, C116, C123
    // VRAM/control, C355, EEPROM, KEYCUS) completes one clock later than before, from a register: the M10K
    // outputs of the widely spread work RAM no longer run through the class mux into the TG68K in the
    // same clock. The SR-2 credit scheduler absorbs the clock (nb1_cpu; measured: docs/M22_TIMING_CLOSURE.md).
    reg  [15:0] dev_rdata;
    always_comb case (cls)
        CLS_RAM1C: dev_rdata = q_1c;
        CLS_SHARE: dev_rdata = q_share;
        CLS_WRAM:  dev_rdata = q_wram;
        CLS_C116:  dev_rdata = q_c116;
        CLS_VRAM:  dev_rdata = q_c123;
        CLS_TMCTL: dev_rdata = q_tmctl;
        CLS_OBJRAM: dev_rdata = q_obj;
        CLS_OBJPOS, CLS_SPRBANK: dev_rdata = q_oreg;
        CLS_EEPROM: dev_rdata = q_eep;
        CLS_KEYCUS: dev_rdata = q_kc;
        CLS_RNG:   dev_rdata = q_misc;
        default:   dev_rdata = 16'hFFFF;
    endcase
    reg         rd1 = 1'b0;         // a registered device read is one clock from done
    reg  [15:0] rd_q = 16'hFFFF;
    always_comb begin
        if (is_write || !rsp_impl || err_q || is_c75ctl || is_iack) bc_rdata = 16'hFFFF;
        // M22: with SYNC_REG the device outputs reach bc_rdata only through rd_q (the per-class arms below
        // are synthesised only for SYNC_REG = 0, so no M10K output path remains into the TG68K)
        else if (SYNC_REG) bc_rdata = is_sync ? rd_q : (cls == CLS_ROM ? rom_word : 16'hFFFF);
        else case (cls)
            CLS_ROM:   bc_rdata = rom_word;
            CLS_RAM1C: bc_rdata = q_1c;
            CLS_SHARE: bc_rdata = q_share;
            CLS_WRAM:  bc_rdata = q_wram;
            CLS_C116:  bc_rdata = q_c116;
            CLS_VRAM:  bc_rdata = q_c123;
            CLS_TMCTL: bc_rdata = q_tmctl;
            CLS_OBJRAM: bc_rdata = q_obj;
            CLS_OBJPOS, CLS_SPRBANK: bc_rdata = q_oreg;
            CLS_EEPROM: bc_rdata = q_eep;
            CLS_KEYCUS: bc_rdata = q_kc;
            CLS_RNG:   bc_rdata = q_misc;
            default:   bc_rdata = 16'hFFFF;
        endcase
    end

    always @(posedge clk_sys) begin
        if (reset) begin
            done_q      <= 1'b0;
            rd1         <= 1'b0;
            filling     <= 1'b0;
            err_q       <= 1'b0;
            lg_mreq_valid <= 1'b0;
            line_valid  <= '0;
            fill_error  <= 1'b0;
        end else begin
            rd1 <= 1'b0;
            if (rd1) begin rd_q <= dev_rdata; done_q <= 1'b1; end
            if (bc_start) begin
                // synchronous device read: data next cycle (M22 SYNC_REG: registered, one more); held sprite write: later
                done_q <= (done_now || (is_sync && !SYNC_REG)) && !obj_wait && !sh_wait;
                rd1    <= SYNC_REG && is_sync && !done_now && !obj_wait && !sh_wait;
                err_q  <= 1'b0;
                if (is_rom && !is_write && !hit) begin
                    // L0 miss: fetch the whole aligned 8-byte line (from SDRAM, or from the M12 L1)
                    filling        <= 1'b1;
                    lg_mreq_valid  <= !PC;
                    lg_mreq_offset <= {5'd0, bc_addr[19:3], 3'b000};
                    done_q         <= 1'b0;
                end
            end
            if (obj_go || sh_go) done_q <= 1'b1;
            if (lg_mreq_valid && mreq_ready) lg_mreq_valid <= 1'b0;
            if (filling && (PC ? pc_done : mrsp_valid)) begin
                filling <= 1'b0;
                done_q  <= 1'b1;
                if (PC ? pc_err : mrsp_err) begin
                    err_q      <= 1'b1;
                    fill_error <= 1'b1;
                end else begin
                    line_data[idx]  <= PC ? pc_line : mrsp_line;
                    line_tag[idx]   <= bc_addr[19:3];
                    line_valid[idx] <= 1'b1;
                end
            end
        end
    end

    // RAM read data arrives one cycle after bc_start (nb1_cpu_ram latency);
    // done_q is set by the bc_start edge, so bc_done and q rise together.

    // ------------------------------------------------------------ C75 control
    // [MAME-SOURCE namconb1_state::cpureg_w case 0x18] bit 0 = 1: HALT clear,
    // RESET assert + clear; bit 0 = 0: HALT. The core has no separate halt
    // state here: a halted C75 is held in reset, which is equivalent because
    // every release pulses reset anyway. CPU reset halts it (MAME's C75 runs
    // from power-on until the 68020's first instruction halts it; holding it
    // instead is harmless, docs/M8_RESEARCH.md).
    always @(posedge clk_sys) begin
        c75_restart <= 1'b0;
        if (reset) begin
            c75_run        <= 1'b0;
            c75_ctl_writes <= '0;
        end else if (bc_start && is_c75ctl && is_write && bc_uds) begin
            c75_run        <= bc_wdata[8];
            c75_restart    <= bc_wdata[8];
            c75_ctl_writes <= c75_ctl_writes + 8'd1;
        end
    end

    // ------------------------------------------------ VBL (M9) and POS (M13) interrupts
    nb1_main_irq main_irq (
        .clk_sys(clk_sys), .reset(reset), .vbl_event(vbl_event),
        .wr_level(bc_start && wr_vbl_level), .wr_ack(bc_start && wr_vbl_ack),
        .wdata(wr_vbl_level ? bc_wdata[15:8] : bc_wdata[7:0]),
        .iack(bc_start && is_iack), .iack_level(bc_addr[3:1]), .ipl(ipl),
        .vbl_level(vbl_level), .vbl_pending(vbl_pending), .vbl_events(vbl_events),
        .vbl_raised(vbl_raised), .vbl_taken(vbl_taken), .vbl_acks(vbl_acks), .level_writes(),
        .pos_event(pos_event), .pos_line(pos_line), .pos_reg5(c116_regs[5*16 +: 16]),
        .wr_pos_level(bc_start && wr_pos_level), .wr_pos_ack(bc_start && wr_pos_ack),
        .pos_wdata(bc_wdata[7:0]), .pos_level(pos_level), .pos_pending(pos_pending),
        .pos_raised(pos_raised), .pos_taken(pos_taken), .pos_acks(pos_acks),
        .pos_frame(pos_frame), .pos_ack_frame(pos_ack_frame), .pos_odd_frames(pos_odd_frames));

    // "after the first VBL": from the first acknowledged VBL interrupt on
    reg vbl_seen = 1'b0;
    always @(posedge clk_sys) begin
        if (reset) begin
            vbl_seen        <= 1'b0;
            iack_cycles     <= '0;
            last_iack_level <= '0;
            pv_unr_valid    <= 1'b0;
            pv_unr_addr     <= '0;
            pv_unw_valid    <= 1'b0;
            pv_unw_addr     <= '0;
            pv_unw_data     <= '0;
            pv_unr_count    <= '0;
        end else if (bc_start) begin
            if (is_iack) begin
                iack_cycles     <= iack_cycles + 32'd1;
                last_iack_level <= bc_addr[3:1];
                if (vbl_level != 4'd0 && bc_addr[3:1] == vbl_level[2:0]) vbl_seen <= 1'b1;
            end else if (vbl_seen && !is_impl) begin
                if (!is_write) begin
                    pv_unr_count <= pv_unr_count + 32'd1;
                    if (!pv_unr_valid) begin
                        pv_unr_valid <= 1'b1;
                        pv_unr_addr  <= byte_addr + {23'd0, ~bc_uds};
                    end
                end else if (cls != CLS_CPUREG && !pv_unw_valid) begin
                    pv_unw_valid <= 1'b1;
                    pv_unw_addr  <= byte_addr + {23'd0, ~bc_uds};
                    pv_unw_data  <= bc_wdata;
                end
            end
        end
    end

    // ------------------------------------------------------------ statistics
    // Last KEYCUS word: a write's data at bc_start, a read's word one cycle
    // later (nb1_keycus registers it). Only this block drives last_kc_data.
    reg kc_rd_q = 1'b0;
    always @(posedge clk_sys) begin
        if (reset) begin
            kc_rd_q      <= 1'b0;
            last_kc_data <= '0;
        end else begin
            kc_rd_q <= bc_start && is_kc && !is_write;
            if (bc_start && is_kc && is_write) last_kc_data <= bc_wdata;
            else if (kc_rd_q)                  last_kc_data <= q_kc;
        end
    end

    always @(posedge clk_sys) begin
        if (reset) begin
            cls_seen           <= '0;
            unimpl_reads       <= '0;
            unimpl_writes      <= '0;
            first_unimpl_valid <= 1'b0;
            first_unimpl_addr  <= '0;
            first_unimpl_we    <= 1'b0;
            last_unimpl_raddr  <= '0;
            last_unimpl_waddr  <= '0;
            last_unimpl_wdata  <= '0;
            rom_writes         <= '0;
            ram_reads          <= '0;
            ram_writes         <= '0;
            c116_reads         <= '0;
            c116_writes        <= '0;
            c123_reads         <= '0;
            c123_writes        <= '0;
            last_c123_addr     <= '0;
            last_c123_we       <= 1'b0;
            eep_reads          <= '0;
            eep_writes         <= '0;
            last_eep_addr      <= '0;
            last_eep_we        <= 1'b0;
            kc_reads           <= '0;
            kc_writes          <= '0;
            last_kc_addr       <= '0;
            last_kc_we         <= 1'b0;
            line_hits          <= '0;
            line_misses        <= '0;
        end else if (bc_start && !is_iack) begin   // an IACK is not a map access
            cls_seen[cls] <= 1'b1;
            if (!is_impl) begin
                if (is_write) begin
                    unimpl_writes     <= unimpl_writes + 32'd1;
                    last_unimpl_waddr <= byte_addr + {23'd0, ~bc_uds};   // odd byte when only D7-0
                    last_unimpl_wdata <= bc_wdata;
                end else begin
                    unimpl_reads      <= unimpl_reads + 32'd1;
                    last_unimpl_raddr <= byte_addr + {23'd0, ~bc_uds};
                end
                if (!first_unimpl_valid) begin
                    first_unimpl_valid <= 1'b1;
                    first_unimpl_addr  <= byte_addr + {23'd0, ~bc_uds};
                    first_unimpl_we    <= is_write;
                end
            end
            if (is_rom && is_write) rom_writes <= rom_writes + 32'd1;
            if (is_ram) begin
                if (is_write) ram_writes <= ram_writes + 32'd1;
                else          ram_reads  <= ram_reads + 32'd1;
            end
            if (is_c116) begin
                if (is_write) c116_writes <= c116_writes + 32'd1;
                else          c116_reads  <= c116_reads + 32'd1;
            end
            if (is_c123) begin
                if (is_write) c123_writes <= c123_writes + 32'd1;
                else          c123_reads  <= c123_reads + 32'd1;
                last_c123_addr <= byte_addr + {23'd0, ~bc_uds};
                last_c123_we   <= is_write;
            end
            if (is_eep) begin
                if (is_write) eep_writes <= eep_writes + 32'd1;
                else          eep_reads  <= eep_reads + 32'd1;
                last_eep_addr <= byte_addr + {23'd0, ~bc_uds};
                last_eep_we   <= is_write;
            end
            if (is_kc) begin
                if (is_write) kc_writes <= kc_writes + 32'd1;
                else          kc_reads  <= kc_reads + 32'd1;
                last_kc_addr <= byte_addr + {23'd0, ~bc_uds};
                last_kc_we   <= is_write;
            end
            if (is_rom && !is_write) begin
                if (hit)      line_hits   <= line_hits + 32'd1;
                else if (!PC) line_misses <= line_misses + 32'd1;
            end
        end
        // M12: L1 hit = a hit; L1 miss = an SDRAM program read (as the M3-M11 miss)
        if (!reset && PC) begin
            if (pc_hit)  line_hits   <= line_hits + 32'd1;
            if (pc_miss) line_misses <= line_misses + 32'd1;
        end
    end
endmodule
