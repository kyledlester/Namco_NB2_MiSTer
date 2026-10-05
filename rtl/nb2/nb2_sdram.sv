// Namco NB-2 MiSTer core -- pipelined SDRAM controller (32 MiB MiSTer module, x16, CL2, BL4).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Why a new controller: the NB-2 ROZ + tile renderers need ~400 random 8-byte reads per 6,144-clock video line
// (docs/MILESTONES.md, video model demand). The NB-1 core's controller (rtl/vendor/sdram.sv, from Sorgelig's
// MiSTer sdram.sv) completes one access every ~8 clocks at best and ~14 behind the NB-1 front end. This one
// overlaps the ACTIVE of the next access with the burst of the current one, across the four banks.
//
// The physical interface is the vendored controller's, unchanged, so the NB-1 SDC constraints and its
// hardware-proven capture timing apply: commands and the address are registered on clk_sys; SDRAM_CLK is
// clk_sys inverted (altddio_out); read word k of a burst whose READ was registered on edge e is in dq_q on edge
// e+4+k (captured from SDRAM_DQ on edge e+3+k). {DQMH,DQML} = SDRAM_A[12:11] at all times, exactly as the
// vendored controller drives them (so it is also correct on boards that tie DQM to A11/A12).
//
// Access protocol (per access, auto-precharge, bank closed afterwards):
//   ACTIVE (row) -> >= tRCD (2) -> READ/WRITE with A10 = 1. Read: words at edges e+4..e+7.
//   Write: one 16-bit word, DQM = ~byte enables on the WRITE cycle.
// Timing rules enforced (clk = 10.33 ns; -6/-7 grade numbers rounded up):
//   tRCD 2, tRRD 2 (ACT->ACT), tRC: ACT->ACT same bank >= 8 here (read col+6 / write col+5, see bank_rdy),
//   tRAS: read col >= ACT+2 (precharge at col+4 >= ACT+6); write col >= ACT+3 (precharge at col+2+tWR),
//   reads >= 4 apart (BL4 on the data bus), write >= 7 after a read (bus turnaround), refresh with every bank
//   idle, tRFC 7.
// DQM rule: read data of a READ on edge c leaves the chip at edges c+2..c+5 and is masked by DQM two clocks
//   earlier, i.e. by A[12:11] on edges c..c+3. So in the 3 cycles after a READ no command may put a non-zero
//   value on A[12:11]: an ACTIVE whose row has A12/A11 set waits (rd_dqm counter).
//
// Address map (bank interleave on consecutive 8-byte blocks): word address w = byte[24:1]
//   bank = byte[4:3], column = {byte[11:5], byte[2:1]}, row = byte[24:12].
//
// Request port: one request per accepted handshake (req_valid && req_ready); reads return a 4-word line
// (rsp_line = {word0, word1, word2, word3}, word0 = the 16-bit word at the 8-byte-aligned address), writes
// return rsp_valid with rsp_write = 1 when the WRITE has been issued. Responses come back in request order
// with the request's tag. Up to 3 requests may be in flight (accepted, not answered).
module nb2_sdram #(
    parameter int TW = 4,                        // tag width
    parameter [13:0] CYCLES_PER_REFRESH = 14'd755 // 64 ms / 8192 rows at 96.768 MHz
) (
    input  wire          clk,
    input  wire          init,                   // (re)initialise: startup sequence, forget requests

    input  wire          req_valid,
    output wire          req_ready,
    input  wire          req_we,
    input  wire [24:1]   req_addr,               // word address; reads use [24:3]
    input  wire [15:0]   req_wdata,
    input  wire [1:0]    req_be,                 // [1] = DQ15..8
    input  wire [TW-1:0] req_tag,

    output reg           rsp_valid = 1'b0,
    output reg           rsp_write = 1'b0,
    output reg  [TW-1:0] rsp_tag = '0,
    output reg  [63:0]   rsp_line = '0,

    output reg           ready = 1'b0,           // startup done

    inout  wire [15:0]   SDRAM_DQ,
    output reg  [12:0]   SDRAM_A = '0,
    output wire          SDRAM_DQML,
    output wire          SDRAM_DQMH,
    output reg  [1:0]    SDRAM_BA = '0,
    output wire          SDRAM_nCS,
    output wire          SDRAM_nWE,
    output wire          SDRAM_nRAS,
    output wire          SDRAM_nCAS,
    output wire          SDRAM_CKE,
    output wire          SDRAM_CLK
);
    localparam [2:0] CMD_NOP = 3'b111, CMD_ACTIVE = 3'b011, CMD_READ = 3'b101, CMD_WRITE = 3'b100,
                     CMD_PRECHARGE = 3'b010, CMD_REFRESH = 3'b001, CMD_LOAD_MODE = 3'b000;
    // CL2, sequential BL4, single-location writes (as the vendored controller)
    localparam [12:0] MODE = {3'b000, 1'b1, 2'b00, 3'd2, 1'b0, 3'b010};

    reg [2:0] command = CMD_NOP;
    assign SDRAM_nCS  = 1'b0;
    assign {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = command;
    assign SDRAM_CKE  = 1'b1;
    assign {SDRAM_DQMH, SDRAM_DQML} = SDRAM_A[12:11];

    reg [15:0] dq_out = '0;
    reg        dq_oe = 1'b0;
    assign SDRAM_DQ = dq_oe ? dq_out : 16'hZZZZ;
    reg [15:0] dq_q = '0;
    always @(posedge clk) dq_q <= SDRAM_DQ;

    // ------------------------------------------------------------------ startup
    // ~125 us of NOPs, then PRECHARGE all, 2 x REFRESH, LOAD MODE (vendored controller order and spacing)
    reg [14:0] boot = '0;
    localparam [14:0] BOOT_END = 15'd12160;
    // ------------------------------------------------------------------ request stages
    // A: accepted, waiting for its ACTIVE. B: activated, waiting for its column command.
    reg          a_v = 1'b0, a_we = 1'b0;
    reg [24:1]   a_addr = '0;
    reg [15:0]   a_wd = '0;
    reg [1:0]    a_be = '0;
    reg [TW-1:0] a_tag = '0;
    reg          b_v = 1'b0, b_we = 1'b0;
    reg [1:0]    b_bank = '0;
    reg [8:0]    b_col = '0;
    reg [15:0]   b_wd = '0;
    reg [1:0]    b_be = '0;
    reg [TW-1:0] b_tag = '0;
    reg [1:0]    b_rcd = '0;                     // cycles until the column command may issue

    wire [1:0]  a_bank = a_addr[4:3];
    wire [12:0] a_row  = a_addr[24:12];
    wire [8:0]  a_col  = {a_addr[11:5], a_addr[2:1]};

    // timers
    reg [3:0] bank_rdy [0:3];
    reg [1:0] rrd = '0;                          // ACT -> ACT
    reg [2:0] colgap = '0;                       // until the next READ may issue
    reg [2:0] wrgap = '0;                        // until the next WRITE may issue
    reg [1:0] rd_dqm = '0;                       // cycles in which A[12:11] must stay 00
    reg [13:0] rf_cnt = '0;
    reg [1:0]  rf_owed = '0;                     // refreshes due (saturating at 3)
    integer i;
    initial for (i = 0; i < 4; i++) bank_rdy[i] = '0;

    // read return pipeline: one entry per READ, data captured on edges e+4..e+7
    reg [7:0]    rp_v = '0;                      // rp_v[k]: a READ was issued k+1 edges ago
    reg [TW-1:0] rp_tag [0:7];
    reg [63:0]   cap = '0;
    reg [2:0]    inflight = '0;                  // accepted (A or B or reading), not answered

    wire all_idle = (bank_rdy[0] == 0) && (bank_rdy[1] == 0) && (bank_rdy[2] == 0) && (bank_rdy[3] == 0);
    assign req_ready = ready && !a_v && (inflight < 3'd3) && !init;

    wire b_col_ok = b_v && (b_rcd == 2'd0) && (b_we ? (wrgap == 3'd0) : (colgap == 3'd0));
    // a refresh that is due stops new ACTIVEs until it has been issued
    wire a_act_ok = a_v && !b_v && (rf_owed == 2'd0) && (bank_rdy[a_bank] == 4'd0) && (rrd == 2'd0) &&
                    ((a_row[12:11] == 2'b00) || (rd_dqm == 2'd0));
    wire rf_go    = (rf_owed != 2'd0) && !b_v && all_idle && (rd_dqm == 2'd0);
    wire rf_inc   = ready && (rf_cnt >= CYCLES_PER_REFRESH);

    always @(posedge clk) begin
        command <= CMD_NOP;
        dq_oe   <= 1'b0;
        rsp_valid <= 1'b0;
        rsp_write <= 1'b0;
        if (rrd != 0)    rrd <= rrd - 2'd1;
        if (colgap != 0) colgap <= colgap - 3'd1;
        if (wrgap != 0)  wrgap <= wrgap - 3'd1;
        if (rd_dqm != 0) rd_dqm <= rd_dqm - 2'd1;
        if (b_rcd != 0)  b_rcd <= b_rcd - 2'd1;
        for (i = 0; i < 4; i++) if (bank_rdy[i] != 0) bank_rdy[i] <= bank_rdy[i] - 4'd1;

        // read data return (edge e+4+k: word k, rp_v[3] = 4 edges after the READ)
        rp_v <= {rp_v[6:0], 1'b0};
        for (i = 7; i > 0; i--) rp_tag[i] <= rp_tag[i-1];
        if (rp_v[3]) cap[63:48] <= dq_q;
        if (rp_v[4]) cap[47:32] <= dq_q;
        if (rp_v[5]) cap[31:16] <= dq_q;
        if (rp_v[6]) begin
            rsp_valid <= 1'b1;
            rsp_tag   <= rp_tag[6];
            rsp_line  <= {cap[63:16], dq_q};
        end

        // refresh bookkeeping
        if (ready) rf_cnt <= rf_inc ? 14'd0 : rf_cnt + 14'd1;
        rf_owed <= rf_owed + ((rf_inc && rf_owed != 2'd3) ? 2'd1 : 2'd0) - ((ready && !b_col_ok && !a_act_ok && rf_go) ? 2'd1 : 2'd0);

        if (!ready) begin
            // ---------------------------------------------------- startup sequence
            boot <= boot + 15'd1;
            SDRAM_A  <= '0;
            SDRAM_BA <= '0;
            case (boot)
                15'd12100: begin command <= CMD_PRECHARGE; SDRAM_A[10] <= 1'b1; end
                15'd12108: command <= CMD_REFRESH;
                15'd12116: command <= CMD_REFRESH;
                15'd12124: begin command <= CMD_LOAD_MODE; SDRAM_A <= MODE; end
                default: ;
            endcase
            if (boot == BOOT_END) ready <= 1'b1;
        end else begin
            // ---------------------------------------------------- accept
            if (req_valid && req_ready) begin
                a_v <= 1'b1; a_we <= req_we; a_addr <= req_addr; a_wd <= req_wdata; a_be <= req_be; a_tag <= req_tag;
            end
            // ---------------------------------------------------- one command per cycle
            if (b_col_ok) begin
                SDRAM_BA <= b_bank;
                if (b_we) begin
                    command  <= CMD_WRITE;
                    SDRAM_A  <= {~b_be, 1'b1, 1'b0, b_col};
                    dq_out   <= b_wd;
                    dq_oe    <= 1'b1;
                    bank_rdy[b_bank] <= 4'd4;     // ACT again from col+5: tWR 2 + tRP 2 (+1)
                    colgap   <= (colgap > 3'd1) ? colgap : 3'd1;
                    wrgap    <= 3'd1;
                    rsp_valid <= 1'b1;            // write answered when issued (no read can be returning:
                    rsp_write <= 1'b1;            // a write never issues while one is in flight, wrgap >= 7)
                    rsp_tag   <= b_tag;
                end else begin
                    command  <= CMD_READ;
                    SDRAM_A  <= {2'b00, 1'b1, 1'b0, b_col};
                    bank_rdy[b_bank] <= 4'd5;     // ACT again from col+6: burst 4 + tRP 2
                    colgap   <= 3'd3;              // next READ from col+4 (BL4 on the data bus)
                    wrgap    <= 3'd7;              // next WRITE from col+8: after the burst and its response
                    rd_dqm   <= 2'd3;
                    rp_v[0]  <= 1'b1;
                    rp_tag[0] <= b_tag;
                end
                b_v <= 1'b0;
            end else if (a_act_ok) begin
                command  <= CMD_ACTIVE;
                SDRAM_BA <= a_bank;
                SDRAM_A  <= a_row;
                rrd      <= 2'd2 - 2'd1;
                b_v <= 1'b1; b_we <= a_we; b_bank <= a_bank; b_col <= a_col; b_wd <= a_wd; b_be <= a_be;
                b_tag <= a_tag;
                b_rcd <= a_we ? 2'd2 : 2'd1;      // column on ACT+2 (read) / ACT+3 (write, tRAS)
                a_v <= 1'b0;
            end else if (rf_go) begin
                command <= CMD_REFRESH;
                SDRAM_A <= '0;
                for (i = 0; i < 4; i++) bank_rdy[i] <= 4'd6;   // ACT again from +7 (tRFC 60 ns)
            end
        end

        // in-flight count: +1 accepted, -1 answered
        inflight <= inflight + ((req_valid && req_ready) ? 3'd1 : 3'd0) - (((rp_v[6]) || (b_col_ok && b_we && ready)) ? 3'd1 : 3'd0);

        if (init) begin
            ready <= 1'b0; boot <= '0; a_v <= 1'b0; b_v <= 1'b0; rp_v <= '0; inflight <= '0;
            rf_cnt <= '0; rf_owed <= '0; rrd <= '0; colgap <= '0; wrgap <= '0; rd_dqm <= '0;
            for (i = 0; i < 4; i++) bank_rdy[i] <= '0;
        end
    end

    // SDRAM_CLK = inverted clk_sys (vendored controller's altddio_out)
`ifdef SIMULATION
    assign SDRAM_CLK = ~clk;
`else
    altddio_out #(
        .extend_oe_disable("OFF"), .intended_device_family("Cyclone V"), .invert_output("OFF"),
        .lpm_hint("UNUSED"), .lpm_type("altddio_out"), .oe_reg("UNREGISTERED"), .power_up_high("OFF"), .width(1)
    ) sdramclk_ddr (
        .datain_h(1'b0), .datain_l(1'b1), .outclock(clk), .dataout(SDRAM_CLK),
        .aclr(1'b0), .aset(1'b0), .oe(1'b1), .outclocken(1'b1), .sclr(1'b0), .sset(1'b0));
`endif
endmodule
