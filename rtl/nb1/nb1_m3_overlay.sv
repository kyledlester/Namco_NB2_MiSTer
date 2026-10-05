// Namco NB-1 MiSTer core -- M3 CPU diagnostic overlay.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// A text panel (5x7 glyphs in 8x8 cells, 34 columns x 27 rows) over the
// native 288x224 raster at x 8..279, y 8..223. Values are raw hex; their
// meaning and the expected Nebulas Ray values are in
// docs/M3_IMPLEMENTATION.md section 9 (nothing game-specific is decided in
// RTL). Rows:
//    0  CPU  <state>   F <frames since CPU start>        (status colour block)
//    1  PC   <last opcode fetch address>
//    2  OBJ  N <sprites in the displayed list> C <cells> D <dropped sprite lines>
//            F <OBJ reads last frame> H <held sprite-RAM writes, low byte>
//            R <1 = a CPU read hit unbacked sprite RAM>; red on snapshot
//            overflow or an OBJ read error (M11; the M3 VEC row, reset
//            vectors, was retired)
//    3  STP  <TG68K steps> BUS <bus cycles>
//    4  RATE <steps last frame> / <nominal ce_cpu last frame>
//    5  LOST <SR-2 lost credits> MAX <longest bus wait, clk_sys>
//    6  UNR  <last unimplemented read addr>  N <count>
//    7  UNW  <last unimplemented write addr> D <data> N <count>
//    8  1ST  <first unimplemented access addr> R|W, then (M11 closeout)
//            Z <frames with a dropped sprite line> M <most sprite lines dropped
//            in one frame> T <frames with a tile-line underrun>
//    9  CLS  16 cells, one per address class (nb1_cpu_map_pkg order)
//   10  PAL  W <C116 writes> R <C116 reads>
//   11  ROM  H <line hits> M <line misses>
//   12  CACR <cacr>, then (M11 tile-underrun diagnostic; the static VBR field was
//            retired): U <line of the last tile underrun> C <CPU> K <C75> T <tile>
//            S <sprite> SDRAM grants in that line window (line_end to line_end)
//            X <most CPU grants in any line since reset>
//   13  CHK  <ROM check: PASS/FAIL/RUN/NONE>
//   14  VRM  W <C123 VRAM writes> R <C123 VRAM reads>            (M5)
//   15  POS  L <POS level> P <POS pending> N <POS interrupts raised in the last
//            CPU frame> A <POS acks in that frame> X <frames since CPU reset whose
//            count was neither 0 nor 224 or whose acks differed>          (M13;
//            replaces the M5 VLA row). Nebulas Ray: L4 P0 N00E0 A00E0 in POS
//            scenes, L0 N0000 A0000 with POS off (jungle demo), X0000.
//   16  EEP  W <EEPROM writes> R <EEPROM reads>                    (M6)
//   17  ELA  <last EEPROM byte address> R|W                        (M6), then (M18
//            persistence, nb1_nvram) D <dirty: CPU writes not yet saved> E <CPU
//            EEPROM writes, saturating FFF> L <load sessions on index 1: 1 = erased
//            image only, 2 = image + a restored .nvm> S <save sessions> then the last
//            session: L|S (load/save) and its byte count (800 = the whole 2 KiB)
//   18  KEY  W <KEYCUS writes> R <KEYCUS reads>                    (M7)
//   19  KLA  <last KEYCUS byte address> R|W D <word read/written>  (M7)
//   20  KCF  <mode> <ID> <ID word> <changing word> | NONE | BAD    (M7 board record)
//            then (M14) I <P1> <P2> <P3> <P4> <MISC>: the C75 input port bytes,
//            active low (FF = nothing pressed; MISC DF = coin 1, 7F = service,
//            FD = service-mode DIP, BF = test switch, FE = freeze DIP)
//   21  C75  RUN|HLT PC <C75 pc> I0 <INT0 interrupts taken>       (M8)
//   22  HSK  N <$A000 bit-7 writes> A000 <last> A0A0 <last>       (M8 boot handshake)
//   23  C7M  I <C75 instructions> OV <overrun instructions>       (M8), then (M15)
//            S <most C352 voices busy in one sample of the last frame> Q <C352 sample
//            render overruns since reset (a sample tick while the previous was pending)>
//   24  VBL  L <VBL level> P <pending> E <VBL events> I <VBL interrupts taken>  (M9)
//   25  VAK  A <VBL acks> W <first unimpl. write after the first VBL, cpureg
//            excluded> R <first unimpl. read after the first VBL>           (M9)
//   26  VID  L <enabled layer mask> C <CHR fetches last frame> S <SHAPE fetches
//            last frame> U <underrun lines> W <worst line render clk_sys>  (M10)
//            The whole row turns red if the renderer saw an SDRAM read error.
// M9 frame latch: every displayed value is sampled once per frame on `snap`
// (nb1_video_timing vblank_begin, the start of line 224, below the panel),
// so one video frame shows one consistent set of values; before M9 the
// fields were live and fast-changing ones photographed as blended glyphs.
// Row 9 class cells: green = implemented class touched (ROM, RAMs, EEPROM,
// C123 VRAM, C123 control (M10), KEYCUS, C116), orange = unimplemented class
// touched, grey = untouched.
// Pipeline: 4 registered stages (M20: glyph bit split from the colour mux), well inside one 16-cycle pixel.
module nb1_m3_overlay (
    input  wire        clk_sys,
    input  wire [8:0]  x,
    input  wire [8:0]  y,
    input  wire        de,
    input  wire [7:0]  in_r, in_g, in_b,
    input  wire        enable,
    input  wire        snap,            // M9: latch every field below (once per frame)

    input  wire [2:0]  cpu_state,       // 0 held (no ROM) 1 held (ROM check) 2 running 3 stalled 4 error
    input  wire [15:0] frames,
    input  wire [23:0] fetch_pc,
    input  wire [31:0] vec_ssp,
    input  wire [31:0] vec_pc,
    input  wire [31:0] steps,
    input  wire [31:0] bus_cycles,
    input  wire [23:0] steps_last_frame,
    input  wire [23:0] earned_last_frame,
    input  wire [31:0] lost_credits,
    input  wire [15:0] max_mem_stall,
    input  wire [23:0] last_unimpl_raddr,
    input  wire [31:0] unimpl_reads,
    input  wire [23:0] last_unimpl_waddr,
    input  wire [15:0] last_unimpl_wdata,
    input  wire [31:0] unimpl_writes,
    input  wire [23:0] first_unimpl_addr,
    input  wire        first_unimpl_we,
    input  wire        first_unimpl_valid,
    input  wire [15:0] cls_seen,
    input  wire [31:0] c116_writes,
    input  wire [31:0] c116_reads,
    input  wire [31:0] c123_writes,
    input  wire [31:0] c123_reads,
    input  wire [23:0] last_c123_addr,
    input  wire        last_c123_we,
    input  wire [31:0] eep_writes,
    input  wire [31:0] eep_reads,
    input  wire [23:0] last_eep_addr,
    input  wire        last_eep_we,
    input  wire [31:0] kc_writes,
    input  wire [31:0] kc_reads,
    input  wire [23:0] last_kc_addr,
    input  wire        last_kc_we,
    input  wire [15:0] last_kc_data,
    input  wire        kc_cfg_seen,     // an ioctl index-2 record was received
    input  wire        kc_cfg_valid,    // ... and it is valid
    input  wire [7:0]  kc_cfg_mode,
    input  wire [15:0] kc_cfg_id,
    input  wire [3:0]  kc_cfg_id_word,
    input  wire [3:0]  kc_cfg_rnd_word,
    input  wire        c75_running,
    input  wire [23:0] c75_pc,
    input  wire [15:0] c75_int0,
    input  wire [7:0]  c75_hs_sets,
    input  wire [15:0] c75_hs_a000,
    input  wire [15:0] c75_hs_a0a0,
    input  wire [31:0] c75_instr,
    input  wire [15:0] c75_overruns,
    input  wire [31:0] line_hits,
    input  wire [31:0] line_misses,
    input  wire [3:0]  cacr,
    input  wire [31:0] vbr,
    input  wire [7:0]  ud_line,         // M11 diagnostic: last tile-underrun line and its grants
    input  wire [8:0]  ud_cpu,
    input  wire [7:0]  ud_c75,
    input  wire [8:0]  ud_tile,
    input  wire [7:0]  ud_spr,
    input  wire [8:0]  ud_cpu_max,
    input  wire [1:0]  check_state,     // 0 none, 1 running, 2 pass, 3 fail
    input  wire [3:0]  vbl_level,       // M9
    input  wire        vbl_pending,
    input  wire [31:0] vbl_events,
    input  wire [31:0] vbl_taken,
    input  wire [31:0] vbl_acks,
    input  wire        pv_unw_valid,
    input  wire [23:0] pv_unw_addr,
    input  wire        pv_unr_valid,
    input  wire [23:0] pv_unr_addr,
    input  wire [5:0]  vid_layers,      // M10 renderer
    input  wire [15:0] vid_chr,
    input  wire [15:0] vid_shape,
    input  wire [15:0] vid_underruns,
    input  wire [15:0] vid_worst,
    input  wire        vid_error,
    input  wire [8:0]  obj_count,       // M11 sprites
    input  wire [12:0] obj_cells,
    input  wire [15:0] obj_drops,
    input  wire [15:0] obj_fetch,
    input  wire [7:0]  obj_holds,
    input  wire        obj_unbacked,
    input  wire        obj_error,
    input  wire [15:0] obj_drop_frames,
    input  wire [7:0]  obj_drop_max,
    input  wire [15:0] vid_under_frames,
    input  wire [3:0]  pos_level,       // M13 POS interrupt
    input  wire        pos_pending,
    input  wire [8:0]  pos_frame,
    input  wire [8:0]  pos_ack_frame,
    input  wire [15:0] pos_odd_frames,
    input  wire [39:0] in_ports,        // M14 {P1, P2, P3, P4, MISC}
    input  wire [5:0]  snd_voices,      // M15 C352
    input  wire [15:0] snd_overruns,
    input  wire        nv_dirty,        // M18 persistence (nb1_nvram)
    input  wire [11:0] nv_events,
    input  wire [3:0]  nv_loads,
    input  wire [7:0]  nv_saves,
    input  wire [11:0] nv_bytes,
    input  wire [1:0]  nv_action,

    output reg  [7:0]  r = 8'd0,
    output reg  [7:0]  g = 8'd0,
    output reg  [7:0]  b = 8'd0
);
    // character codes: 0-15 hex digits, 16+ letters/symbols
    localparam [5:0] C_SP = 6'd63;
    function automatic [5:0] L(input [7:0] ch);   // ASCII -> code
        if (ch >= "0" && ch <= "9") L = 6'(ch - "0");
        else if (ch >= "A" && ch <= "F") L = 6'(ch - "A" + 10);
        else if (ch >= "G" && ch <= "Z") L = 6'(ch - "G" + 16);
        else if (ch == "/") L = 6'd36;
        else if (ch == ":") L = 6'd37;
        else L = C_SP;
    endfunction

    // 5x7 glyph rows (bit 4 = leftmost column)
    function automatic [4:0] glyph(input [5:0] c, input [2:0] row);
        reg [34:0] g;
        case (c)
            6'd0:  g = 35'b01110_10001_10011_10101_11001_10001_01110;
            6'd1:  g = 35'b00100_01100_00100_00100_00100_00100_01110;
            6'd2:  g = 35'b01110_10001_00001_00010_00100_01000_11111;
            6'd3:  g = 35'b11110_00001_00001_01110_00001_00001_11110;
            6'd4:  g = 35'b00010_00110_01010_10010_11111_00010_00010;
            6'd5:  g = 35'b11111_10000_11110_00001_00001_10001_01110;
            6'd6:  g = 35'b00110_01000_10000_11110_10001_10001_01110;
            6'd7:  g = 35'b11111_00001_00010_00100_01000_01000_01000;
            6'd8:  g = 35'b01110_10001_10001_01110_10001_10001_01110;
            6'd9:  g = 35'b01110_10001_10001_01111_00001_00010_01100;
            6'd10: g = 35'b01110_10001_10001_11111_10001_10001_10001; // A
            6'd11: g = 35'b11110_10001_10001_11110_10001_10001_11110; // B
            6'd12: g = 35'b01110_10001_10000_10000_10000_10001_01110; // C
            6'd13: g = 35'b11100_10010_10001_10001_10001_10010_11100; // D
            6'd14: g = 35'b11111_10000_10000_11110_10000_10000_11111; // E
            6'd15: g = 35'b11111_10000_10000_11110_10000_10000_10000; // F
            6'd16: g = 35'b01110_10001_10000_10111_10001_10001_01111; // G
            6'd17: g = 35'b10001_10001_10001_11111_10001_10001_10001; // H
            6'd18: g = 35'b01110_00100_00100_00100_00100_00100_01110; // I
            6'd19: g = 35'b00111_00010_00010_00010_00010_10010_01100; // J
            6'd20: g = 35'b10001_10010_10100_11000_10100_10010_10001; // K
            6'd21: g = 35'b10000_10000_10000_10000_10000_10000_11111; // L
            6'd22: g = 35'b10001_11011_10101_10101_10001_10001_10001; // M
            6'd23: g = 35'b10001_10001_11001_10101_10011_10001_10001; // N
            6'd24: g = 35'b01110_10001_10001_10001_10001_10001_01110; // O
            6'd25: g = 35'b11110_10001_10001_11110_10000_10000_10000; // P
            6'd26: g = 35'b01110_10001_10001_10001_10101_10010_01101; // Q
            6'd27: g = 35'b11110_10001_10001_11110_10100_10010_10001; // R
            6'd28: g = 35'b01111_10000_10000_01110_00001_00001_11110; // S
            6'd29: g = 35'b11111_00100_00100_00100_00100_00100_00100; // T
            6'd30: g = 35'b10001_10001_10001_10001_10001_10001_01110; // U
            6'd31: g = 35'b10001_10001_10001_10001_10001_01010_00100; // V
            6'd32: g = 35'b10001_10001_10001_10101_10101_10101_01010; // W
            6'd33: g = 35'b10001_10001_01010_00100_01010_10001_10001; // X
            6'd34: g = 35'b10001_10001_01010_00100_00100_00100_00100; // Y
            6'd35: g = 35'b11111_00001_00010_00100_01000_10000_11111; // Z
            6'd36: g = 35'b00001_00010_00010_00100_01000_01000_10000; // /
            6'd37: g = 35'b00000_01100_01100_00000_01100_01100_00000; // :
            default: g = '0;
        endcase
        glyph = g[34 - 5 * row -: 5];
    endfunction

    // R-OVL-1 (fixed in M8): ndig/nd were [2:0], so the 8 of every 8-digit
    // field truncated to 0 and those fields rendered blank on hardware.
    function automatic [5:0] hexd(input [31:0] v, input [3:0] ndig, input [5:0] k);
        // digit k (0 = leftmost) of the ndig-digit hex rendering of v
        reg [5:0] sh;
        sh = 6'((ndig - 1 - k) * 4);
        hexd = 6'((v >> sh) & 32'hF);
    endfunction

    // ---------------------------------------------------------- M9 frame latch
    // Generated from the port list: <field>_f is <field> sampled on snap.
    reg [2:0] cpu_state_f;
    reg [15:0] frames_f;
    reg [23:0] fetch_pc_f;
    reg [31:0] steps_f;
    reg [31:0] bus_cycles_f;
    reg [23:0] steps_last_frame_f;
    reg [23:0] earned_last_frame_f;
    reg [31:0] lost_credits_f;
    reg [15:0] max_mem_stall_f;
    reg [23:0] last_unimpl_raddr_f;
    reg [31:0] unimpl_reads_f;
    reg [23:0] last_unimpl_waddr_f;
    reg [15:0] last_unimpl_wdata_f;
    reg [31:0] unimpl_writes_f;
    reg [23:0] first_unimpl_addr_f;
    reg first_unimpl_we_f;
    reg first_unimpl_valid_f;
    reg [15:0] cls_seen_f;
    reg [31:0] c116_writes_f;
    reg [31:0] c116_reads_f;
    reg [31:0] c123_writes_f;
    reg [31:0] c123_reads_f;
    reg [23:0] last_c123_addr_f;
    reg last_c123_we_f;
    reg [31:0] eep_writes_f;
    reg [31:0] eep_reads_f;
    reg [23:0] last_eep_addr_f;
    reg last_eep_we_f;
    reg nv_dirty_f;
    reg [11:0] nv_events_f, nv_bytes_f;
    reg [3:0] nv_loads_f;
    reg [7:0] nv_saves_f;
    reg [1:0] nv_action_f;
    reg [31:0] kc_writes_f;
    reg [31:0] kc_reads_f;
    reg [23:0] last_kc_addr_f;
    reg last_kc_we_f;
    reg [15:0] last_kc_data_f;
    reg kc_cfg_seen_f;
    reg kc_cfg_valid_f;
    reg [7:0] kc_cfg_mode_f;
    reg [15:0] kc_cfg_id_f;
    reg [3:0] kc_cfg_id_word_f;
    reg [3:0] kc_cfg_rnd_word_f;
    reg c75_running_f;
    reg [23:0] c75_pc_f;
    reg [15:0] c75_int0_f;
    reg [7:0] c75_hs_sets_f;
    reg [15:0] c75_hs_a000_f;
    reg [15:0] c75_hs_a0a0_f;
    reg [31:0] c75_instr_f;
    reg [15:0] c75_overruns_f;
    reg [31:0] line_hits_f;
    reg [31:0] line_misses_f;
    reg [3:0] cacr_f;
    reg [31:0] vbr_f;
    reg [7:0] ud_line_f, ud_c75_f, ud_spr_f;
    reg [8:0] ud_cpu_f, ud_tile_f, ud_cpu_max_f;
    reg [1:0] check_state_f;
    reg [3:0] vbl_level_f;
    reg vbl_pending_f;
    reg [31:0] vbl_events_f;
    reg [31:0] vbl_taken_f;
    reg [31:0] vbl_acks_f;
    reg pv_unw_valid_f;
    reg [23:0] pv_unw_addr_f;
    reg pv_unr_valid_f;
    reg [23:0] pv_unr_addr_f;
    reg [5:0] vid_layers_f;
    reg [15:0] vid_chr_f;
    reg [15:0] vid_shape_f;
    reg [15:0] vid_underruns_f;
    reg [15:0] vid_worst_f;
    reg vid_error_f;
    reg [8:0] obj_count_f;
    reg [12:0] obj_cells_f;
    reg [15:0] obj_drops_f;
    reg [15:0] obj_fetch_f;
    reg [7:0] obj_holds_f;
    reg obj_unbacked_f;
    reg obj_error_f;
    reg [15:0] obj_drop_frames_f;
    reg [3:0] pos_level_f;
    reg pos_pending_f;
    reg [8:0] pos_frame_f, pos_ack_frame_f;
    reg [15:0] pos_odd_frames_f;
    reg [39:0] in_ports_f;
    reg [5:0]  snd_voices_f;
    reg [15:0] snd_overruns_f;
    reg [7:0] obj_drop_max_f;
    reg [15:0] vid_under_frames_f;
    always @(posedge clk_sys) if (snap) begin
        cpu_state_f <= cpu_state;
        frames_f <= frames;
        fetch_pc_f <= fetch_pc;
        steps_f <= steps;
        bus_cycles_f <= bus_cycles;
        steps_last_frame_f <= steps_last_frame;
        earned_last_frame_f <= earned_last_frame;
        lost_credits_f <= lost_credits;
        max_mem_stall_f <= max_mem_stall;
        last_unimpl_raddr_f <= last_unimpl_raddr;
        unimpl_reads_f <= unimpl_reads;
        last_unimpl_waddr_f <= last_unimpl_waddr;
        last_unimpl_wdata_f <= last_unimpl_wdata;
        unimpl_writes_f <= unimpl_writes;
        first_unimpl_addr_f <= first_unimpl_addr;
        first_unimpl_we_f <= first_unimpl_we;
        first_unimpl_valid_f <= first_unimpl_valid;
        cls_seen_f <= cls_seen;
        c116_writes_f <= c116_writes;
        c116_reads_f <= c116_reads;
        c123_writes_f <= c123_writes;
        c123_reads_f <= c123_reads;
        last_c123_addr_f <= last_c123_addr;
        last_c123_we_f <= last_c123_we;
        eep_writes_f <= eep_writes;
        eep_reads_f <= eep_reads;
        last_eep_addr_f <= last_eep_addr;
        last_eep_we_f <= last_eep_we;
        nv_dirty_f <= nv_dirty;
        nv_events_f <= nv_events;
        nv_loads_f <= nv_loads;
        nv_saves_f <= nv_saves;
        nv_bytes_f <= nv_bytes;
        nv_action_f <= nv_action;
        kc_writes_f <= kc_writes;
        kc_reads_f <= kc_reads;
        last_kc_addr_f <= last_kc_addr;
        last_kc_we_f <= last_kc_we;
        last_kc_data_f <= last_kc_data;
        kc_cfg_seen_f <= kc_cfg_seen;
        kc_cfg_valid_f <= kc_cfg_valid;
        kc_cfg_mode_f <= kc_cfg_mode;
        kc_cfg_id_f <= kc_cfg_id;
        kc_cfg_id_word_f <= kc_cfg_id_word;
        kc_cfg_rnd_word_f <= kc_cfg_rnd_word;
        c75_running_f <= c75_running;
        c75_pc_f <= c75_pc;
        c75_int0_f <= c75_int0;
        c75_hs_sets_f <= c75_hs_sets;
        c75_hs_a000_f <= c75_hs_a000;
        c75_hs_a0a0_f <= c75_hs_a0a0;
        c75_instr_f <= c75_instr;
        c75_overruns_f <= c75_overruns;
        line_hits_f <= line_hits;
        line_misses_f <= line_misses;
        cacr_f <= cacr;
        vbr_f <= vbr;
        ud_line_f <= ud_line;
        ud_cpu_f <= ud_cpu;
        ud_c75_f <= ud_c75;
        ud_tile_f <= ud_tile;
        ud_spr_f <= ud_spr;
        ud_cpu_max_f <= ud_cpu_max;
        check_state_f <= check_state;
        vbl_level_f <= vbl_level;
        vbl_pending_f <= vbl_pending;
        vbl_events_f <= vbl_events;
        vbl_taken_f <= vbl_taken;
        vbl_acks_f <= vbl_acks;
        pv_unw_valid_f <= pv_unw_valid;
        pv_unw_addr_f <= pv_unw_addr;
        pv_unr_valid_f <= pv_unr_valid;
        pv_unr_addr_f <= pv_unr_addr;
        vid_layers_f <= vid_layers;
        vid_chr_f <= vid_chr;
        vid_shape_f <= vid_shape;
        vid_underruns_f <= vid_underruns;
        vid_worst_f <= vid_worst;
        vid_error_f <= vid_error;
        obj_count_f <= obj_count;
        obj_cells_f <= obj_cells;
        obj_drops_f <= obj_drops;
        obj_fetch_f <= obj_fetch;
        obj_holds_f <= obj_holds;
        obj_unbacked_f <= obj_unbacked;
        obj_error_f <= obj_error;
        obj_drop_frames_f <= obj_drop_frames;
        obj_drop_max_f <= obj_drop_max;
        vid_under_frames_f <= vid_under_frames;
        pos_level_f <= pos_level;
        pos_pending_f <= pos_pending;
        pos_frame_f <= pos_frame;
        pos_ack_frame_f <= pos_ack_frame;
        pos_odd_frames_f <= pos_odd_frames;
        in_ports_f <= in_ports;
        snd_voices_f <= snd_voices;
        snd_overruns_f <= snd_overruns;
    end

    // ---------------------------------------------------------- stage 1
    wire       panel = (x >= 9'd8) && (x < 9'd280) && (y >= 9'd8) && (y < 9'd224);
    wire [8:0] px = x - 9'd8;
    wire [8:0] py = y - 9'd8;
    reg  [5:0] s1_col;
    reg  [4:0] s1_row;
    reg  [2:0] s1_gx, s1_gy;
    reg        s1_panel, s1_de;
    reg  [23:0] s1_bg;
    always @(posedge clk_sys) begin
        s1_col   <= px[8:3];
        s1_row   <= py[7:3];
        s1_gx    <= px[2:0];
        s1_gy    <= py[2:0];
        s1_panel <= panel && enable;
        s1_de    <= de;
        s1_bg    <= {in_r, in_g, in_b};
    end

    // ---------------------------------------------------------- stage 2: char
    // text(row, col): label in columns 0-4, fields after.
    function automatic [5:0] lbl(input [39:0] s, input [5:0] col);   // 5-char label
        lbl = (col < 6'd5) ? L(s[39 - 8 * col -: 8]) : C_SP;
    endfunction
    function automatic [5:0] field(input [31:0] v, input [3:0] nd, input [5:0] col, input [5:0] start);
        field = (col >= start && col < start + 6'(nd)) ? hexd(v, nd, col - start) : C_SP;
    endfunction

    reg [5:0]  s2_ch;
    reg [23:0] s2_fg;
    reg [2:0]  s2_gx, s2_gy;
    reg        s2_panel, s2_de, s2_cell;
    reg [23:0] s2_bg, s2_cellc;
    reg [5:0]  c;
    reg [23:0] stc;
    always_comb begin
        case (cpu_state_f)
            3'd0:    stc = 24'h404040;   // held: no ROM loaded
            3'd1:    stc = 24'h2040FF;   // held: ROM check running
            3'd2:    stc = 24'h00E000;   // running (steps_f advancing)
            3'd3:    stc = 24'hFFC000;   // running, no step for a whole frame
            default: stc = 24'hFF0000;   // error (line fill error)
        endcase
        c = C_SP;
        case (s1_row)
            5'd0:  c = (s1_col < 5) ? lbl("CPU  ", s1_col) : (s1_col == 8 ? L("F") : field(frames_f, 4, s1_col, 10));
            5'd1:  c = (s1_col < 5) ? lbl("PC   ", s1_col) : field(fetch_pc_f, 6, s1_col, 6);
            5'd2:  c = (s1_col < 5) ? lbl("OBJ  ", s1_col) :
                        (s1_col == 5 ? L("N") : (s1_col >= 6 && s1_col < 9) ? field({23'd0, obj_count_f}, 3, s1_col, 6) :
                        (s1_col == 10 ? L("C") : (s1_col >= 11 && s1_col < 15) ? field({19'd0, obj_cells_f}, 4, s1_col, 11) :
                        (s1_col == 16 ? L("D") : (s1_col >= 17 && s1_col < 21) ? field({16'd0, obj_drops_f}, 4, s1_col, 17) :
                        (s1_col == 22 ? L("F") : (s1_col >= 23 && s1_col < 27) ? field({16'd0, obj_fetch_f}, 4, s1_col, 23) :
                        (s1_col == 28 ? L("H") : (s1_col >= 29 && s1_col < 31) ? field({24'd0, obj_holds_f}, 2, s1_col, 29) :
                        (s1_col == 32 ? L("R") : field({31'd0, obj_unbacked_f}, 1, s1_col, 33)))))));
            5'd3:  c = (s1_col < 5) ? lbl("STP  ", s1_col) : (s1_col < 15 ? field(steps_f, 8, s1_col, 6) :
                        (s1_col < 19 ? lbl("BUS  ", s1_col - 15) : field(bus_cycles_f, 8, s1_col, 19)));
            5'd4:  c = (s1_col < 5) ? lbl("RATE ", s1_col) : (s1_col < 13 ? field({8'd0, steps_last_frame_f}, 6, s1_col, 6) :
                        (s1_col == 13 ? L("/") : field({8'd0, earned_last_frame_f}, 6, s1_col, 15)));
            5'd5:  c = (s1_col < 5) ? lbl("LOST ", s1_col) : (s1_col < 15 ? field(lost_credits_f, 8, s1_col, 6) :
                        (s1_col < 19 ? lbl("MAX  ", s1_col - 15) : field({16'd0, max_mem_stall_f}, 4, s1_col, 19)));
            5'd6:  c = (s1_col < 5) ? lbl("UNR  ", s1_col) : (s1_col < 13 ? field({8'd0, last_unimpl_raddr_f}, 6, s1_col, 6) :
                        (s1_col == 14 ? L("N") : field(unimpl_reads_f, 8, s1_col, 16)));
            5'd7:  c = (s1_col < 5) ? lbl("UNW  ", s1_col) : (s1_col < 13 ? field({8'd0, last_unimpl_waddr_f}, 6, s1_col, 6) :
                        (s1_col == 13 ? L("D") : (s1_col < 19 ? field({16'd0, last_unimpl_wdata_f}, 4, s1_col, 14) :
                        (s1_col == 20 ? L("N") : field(unimpl_writes_f, 8, s1_col, 22)))));
            5'd8:  c = (s1_col < 5) ? lbl("1ST  ", s1_col) :
                        (s1_col < 15 ? (!first_unimpl_valid_f ? C_SP :
                            (s1_col < 13 ? field({8'd0, first_unimpl_addr_f}, 6, s1_col, 6) :
                            (s1_col == 13 ? (first_unimpl_we_f ? L("W") : L("R")) : C_SP))) :
                        (s1_col == 15 ? L("Z") : (s1_col >= 16 && s1_col < 20) ? field({16'd0, obj_drop_frames_f}, 4, s1_col, 16) :
                        (s1_col == 21 ? L("M") : (s1_col >= 22 && s1_col < 24) ? field({24'd0, obj_drop_max_f}, 2, s1_col, 22) :
                        (s1_col == 25 ? L("T") : field({16'd0, vid_under_frames_f}, 4, s1_col, 26)))));
            5'd9:  c = (s1_col < 5) ? lbl("CLS  ", s1_col) : C_SP;
            5'd10: c = (s1_col < 5) ? lbl("PAL  ", s1_col) : (s1_col == 5 ? L("W") : (s1_col < 15 ? field(c116_writes_f, 8, s1_col, 6) :
                        (s1_col == 15 ? L("R") : field(c116_reads_f, 8, s1_col, 16))));
            5'd11: c = (s1_col < 5) ? lbl("ROM  ", s1_col) : (s1_col == 5 ? L("H") : (s1_col < 15 ? field(line_hits_f, 8, s1_col, 6) :
                        (s1_col == 15 ? L("M") : field(line_misses_f, 8, s1_col, 16))));
            5'd12: c = (s1_col < 5) ? lbl("CACR ", s1_col) : (s1_col < 8 ? field({28'd0, cacr_f}, 1, s1_col, 6) :
                        (s1_col == 8 ? L("U") : (s1_col < 11 ? field({24'd0, ud_line_f}, 2, s1_col, 9) :
                        (s1_col == 12 ? L("C") : (s1_col < 16 ? field({23'd0, ud_cpu_f}, 3, s1_col, 13) :
                        (s1_col == 17 ? L("K") : (s1_col < 20 ? field({24'd0, ud_c75_f}, 2, s1_col, 18) :
                        (s1_col == 21 ? L("T") : (s1_col < 25 ? field({23'd0, ud_tile_f}, 3, s1_col, 22) :
                        (s1_col == 26 ? L("S") : (s1_col < 29 ? field({24'd0, ud_spr_f}, 2, s1_col, 27) :
                        (s1_col == 30 ? L("X") : field({23'd0, ud_cpu_max_f}, 3, s1_col, 31)))))))))))));
            5'd13: c = (s1_col < 5) ? lbl("CHK  ", s1_col) : (s1_col < 6 ? C_SP :
                        (check_state_f == 2'd2 ? lbl("PASS ", s1_col - 6) : check_state_f == 2'd3 ? lbl("FAIL ", s1_col - 6) :
                         check_state_f == 2'd1 ? lbl("RUN  ", s1_col - 6) : lbl("NONE ", s1_col - 6)));
            5'd14: c = (s1_col < 5) ? lbl("VRM  ", s1_col) : (s1_col == 5 ? L("W") : (s1_col < 15 ? field(c123_writes_f, 8, s1_col, 6) :
                        (s1_col == 15 ? L("R") : field(c123_reads_f, 8, s1_col, 16))));
            5'd15: c = (s1_col < 5) ? lbl("POS  ", s1_col) :
                        (s1_col == 5 ? L("L") : s1_col == 6 ? field({28'd0, pos_level_f}, 1, s1_col, 6) :
                        (s1_col == 8 ? L("P") : s1_col == 9 ? field({31'd0, pos_pending_f}, 1, s1_col, 9) :
                        (s1_col == 11 ? L("N") : (s1_col >= 12 && s1_col < 16) ? field({23'd0, pos_frame_f}, 4, s1_col, 12) :
                        (s1_col == 17 ? L("A") : (s1_col >= 18 && s1_col < 22) ? field({23'd0, pos_ack_frame_f}, 4, s1_col, 18) :
                        (s1_col == 23 ? L("X") : field({16'd0, pos_odd_frames_f}, 4, s1_col, 24))))));
            5'd16: c = (s1_col < 5) ? lbl("EEP  ", s1_col) : (s1_col == 5 ? L("W") : (s1_col < 15 ? field(eep_writes_f, 8, s1_col, 6) :
                        (s1_col == 15 ? L("R") : field(eep_reads_f, 8, s1_col, 16))));
            5'd17: c = (s1_col < 5) ? lbl("ELA  ", s1_col) : (s1_col < 13 ? field({8'd0, last_eep_addr_f}, 6, s1_col, 6) :
                        (s1_col == 13 ? ((eep_reads_f | eep_writes_f) == 32'd0 ? C_SP : (last_eep_we_f ? L("W") : L("R"))) :
                        (s1_col == 15 ? L("D") : s1_col == 16 ? field({31'd0, nv_dirty_f}, 1, s1_col, 16) :
                        (s1_col == 18 ? L("E") : (s1_col >= 19 && s1_col < 22) ? field({20'd0, nv_events_f}, 3, s1_col, 19) :
                        (s1_col == 23 ? L("L") : s1_col == 24 ? field({28'd0, nv_loads_f}, 1, s1_col, 24) :
                        (s1_col == 26 ? L("S") : (s1_col >= 27 && s1_col < 29) ? field({24'd0, nv_saves_f}, 2, s1_col, 27) :
                        (s1_col == 30 ? (nv_action_f == 2'd1 ? L("L") : nv_action_f == 2'd2 ? L("S") : C_SP) :
                        (s1_col >= 31 && s1_col < 34) ? field({20'd0, nv_bytes_f}, 3, s1_col, 31) : C_SP)))))));
            5'd18: c = (s1_col < 5) ? lbl("KEY  ", s1_col) : (s1_col == 5 ? L("W") : (s1_col < 15 ? field(kc_writes_f, 8, s1_col, 6) :
                        (s1_col == 15 ? L("R") : field(kc_reads_f, 8, s1_col, 16))));
            5'd19: c = (s1_col < 5) ? lbl("KLA  ", s1_col) : ((kc_reads_f | kc_writes_f) == 32'd0 ? C_SP :
                        (s1_col < 13 ? field({8'd0, last_kc_addr_f}, 6, s1_col, 6) :
                        (s1_col == 13 ? (last_kc_we_f ? L("W") : L("R")) :
                        (s1_col == 15 ? L("D") : field({16'd0, last_kc_data_f}, 4, s1_col, 17)))));
            5'd20: begin c = (s1_col < 5) ? lbl("KCF  ", s1_col) : (s1_col < 6 ? C_SP :
                        (!kc_cfg_seen_f ? lbl("NONE ", s1_col - 6) : !kc_cfg_valid_f ? lbl("BAD  ", s1_col - 6) :
                        (s1_col < 8 ? field({24'd0, kc_cfg_mode_f}, 2, s1_col, 6) :
                        (s1_col < 13 ? field({16'd0, kc_cfg_id_f}, 4, s1_col, 9) :
                        (s1_col == 14 ? field({28'd0, kc_cfg_id_word_f}, 1, s1_col, 14) :
                        (s1_col == 16 ? field({28'd0, kc_cfg_rnd_word_f}, 1, s1_col, 16) : C_SP))))));
            if (s1_row == 5'd20 && s1_col >= 6'd18)
                c = (s1_col == 18) ? L("I") :
                    (s1_col < 22) ? field({24'd0, in_ports_f[39:32]}, 2, s1_col, 20) :
                    (s1_col < 25) ? field({24'd0, in_ports_f[31:24]}, 2, s1_col, 23) :
                    (s1_col < 28) ? field({24'd0, in_ports_f[23:16]}, 2, s1_col, 26) :
                    (s1_col < 31) ? field({24'd0, in_ports_f[15:8]},  2, s1_col, 29) :
                                    field({24'd0, in_ports_f[7:0]},   2, s1_col, 32);
            end
            5'd21: c = (s1_col < 5) ? lbl("C75  ", s1_col) :
                        (s1_col < 6 ? C_SP : s1_col < 9 ? (c75_running_f ? lbl("RUN  ", s1_col - 6) : lbl("HLT  ", s1_col - 6)) :
                        (s1_col == 10 ? L("P") : s1_col == 11 ? L("C") :
                        (s1_col >= 13 && s1_col < 19) ? field({8'd0, c75_pc_f}, 6, s1_col, 13) :
                        (s1_col == 20 ? L("I") : s1_col == 21 ? 6'd0 : field({16'd0, c75_int0_f}, 4, s1_col, 23))));
            5'd22: c = (s1_col < 5) ? lbl("HSK  ", s1_col) :
                        (s1_col == 6 ? L("N") : (s1_col == 8 || s1_col == 9) ? field({24'd0, c75_hs_sets_f}, 2, s1_col, 8) :
                        (s1_col >= 11 && s1_col < 15) ? lbl("A000 ", s1_col - 11) :
                        (s1_col >= 16 && s1_col < 20) ? field({16'd0, c75_hs_a000_f}, 4, s1_col, 16) :
                        (s1_col >= 21 && s1_col < 25) ? lbl("A0A0 ", s1_col - 21) :
                        field({16'd0, c75_hs_a0a0_f}, 4, s1_col, 26));
            5'd23: c = (s1_col < 5) ? lbl("C7M  ", s1_col) :
                        (s1_col == 5) ? L("I") : (s1_col < 15) ? field(c75_instr_f, 8, s1_col, 6) :
                        (s1_col == 16) ? L("O") : (s1_col == 17) ? L("V") :
                        (s1_col < 24) ? field({16'd0, c75_overruns_f}, 4, s1_col, 19) :
                        (s1_col == 24) ? L("S") : (s1_col < 28) ? field({26'd0, snd_voices_f}, 2, s1_col, 25) :
                        (s1_col == 28) ? L("Q") : field({16'd0, snd_overruns_f}, 4, s1_col, 29);
            5'd24: c = (s1_col < 5) ? lbl("VBL  ", s1_col) :
                        (s1_col == 5 ? L("L") : s1_col == 6 ? field({28'd0, vbl_level_f}, 1, s1_col, 6) :
                        (s1_col == 8 ? L("P") : s1_col == 9 ? field({31'd0, vbl_pending_f}, 1, s1_col, 9) :
                        (s1_col == 11 ? L("E") : (s1_col < 22 ? field(vbl_events_f, 8, s1_col, 13) :
                        (s1_col == 22 ? L("I") : field(vbl_taken_f, 8, s1_col, 24))))));
            5'd25: c = (s1_col < 5) ? lbl("VAK  ", s1_col) :
                        (s1_col == 5 ? L("A") : (s1_col < 16 ? field(vbl_acks_f, 8, s1_col, 7) :
                        (s1_col == 16 ? L("W") : (s1_col < 25 ? (pv_unw_valid_f ? field({8'd0, pv_unw_addr_f}, 6, s1_col, 18) : C_SP) :
                        (s1_col == 25 ? L("R") : (pv_unr_valid_f ? field({8'd0, pv_unr_addr_f}, 6, s1_col, 27) : C_SP))))));
            5'd26: c = (s1_col < 5) ? lbl("VID  ", s1_col) :
                        (s1_col == 5 ? L("L") : (s1_col == 6 || s1_col == 7) ? field({26'd0, vid_layers_f}, 2, s1_col, 6) :
                        (s1_col == 9 ? L("C") : (s1_col >= 10 && s1_col < 14) ? field({16'd0, vid_chr_f}, 4, s1_col, 10) :
                        (s1_col == 15 ? L("S") : (s1_col >= 16 && s1_col < 20) ? field({16'd0, vid_shape_f}, 4, s1_col, 16) :
                        (s1_col == 21 ? L("U") : (s1_col >= 22 && s1_col < 26) ? field({16'd0, vid_underruns_f}, 4, s1_col, 22) :
                        (s1_col == 27 ? L("W") : field({16'd0, vid_worst_f}, 4, s1_col, 28))))));
            default: c = C_SP;
        endcase
    end

    always @(posedge clk_sys) begin
        s2_ch    <= c;
        s2_gx    <= s1_gx;
        s2_gy    <= s1_gy;
        s2_panel <= s1_panel;
        s2_de    <= s1_de;
        s2_bg    <= s1_bg;
        s2_fg    <= ((s1_row == 5'd26 && vid_error_f) || (s1_row == 5'd2 && obj_error_f)) ? 24'hFF4040 : 24'hFFFFFF;
        // row 0: status block in columns 5-6; row 9: class cells from column 6
        s2_cell  <= 1'b0;
        s2_cellc <= 24'h000000;
        if (s1_row == 5'd0 && (s1_col == 6'd5 || s1_col == 6'd6)) begin
            s2_cell <= 1'b1; s2_cellc <= stc;
        end
        if (s1_row == 5'd9 && s1_col >= 6'd6 && s1_col < 6'd22 && s1_gx < 3'd6 && s1_gy < 3'd7) begin
            s2_cell  <= 1'b1;
            s2_cellc <= !cls_seen_f[s1_col - 6'd6] ? 24'h303030 :
                        ((s1_col - 6'd6 < 6'd4 || s1_col - 6'd6 == 6'd7 || s1_col - 6'd6 == 6'd8 || s1_col - 6'd6 == 6'd9 || s1_col - 6'd6 == 6'd10 || s1_col - 6'd6 == 6'd11 || s1_col - 6'd6 == 6'd12 || s1_col - 6'd6 == 6'd13 || s1_col - 6'd6 == 6'd14) ? 24'h00C000 : 24'hFF8000);
        end
    end

    // ---------------------------------------------------------- stage 3: glyph bit (M20 timing: the glyph
    // lookup is registered apart from the colour mux; 4 stages, still well inside one 16-clk pixel)
    reg        s3_on = 1'b0, s3_panel = 1'b0, s3_de = 1'b0, s3_cell = 1'b0;
    reg [23:0] s3_fg = '0, s3_bg = '0, s3_cellc = '0;
    always @(posedge clk_sys) begin : st3
        reg [4:0] gl;
        gl = glyph(s2_ch, s2_gy);
        s3_on    <= (s2_gx < 3'd5) && (s2_gy < 3'd7) && gl[3'd4 - s2_gx];
        s3_panel <= s2_panel; s3_de <= s2_de; s3_cell <= s2_cell;
        s3_fg    <= s2_fg;    s3_bg <= s2_bg; s3_cellc <= s2_cellc;
    end
    // ---------------------------------------------------------- stage 4: pixel
    always @(posedge clk_sys) begin
        if (s3_de && s3_panel) {r, g, b} <= s3_cell ? s3_cellc : (s3_on ? s3_fg : 24'h000000);
        else                   {r, g, b} <= s3_bg;
    end
endmodule
