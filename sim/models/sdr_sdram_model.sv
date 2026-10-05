// Behavioural SDR SDRAM model (32 MiB, x16, 4 banks, 8192 rows x 512 columns: the standard MiSTer module).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// From the NB-1 core's development bench model (sim/models/sdr_sdram_model.sv in Namco_NB1_Mister). NB-2
// changes (STRICT = 1, the default here): auto-precharge recovery (tRAS, tWR + tRP), tRRD and tRC violations
// are errors, not counted notes; DQM is applied to read data with the datasheet latency of 2 (a masked word
// is not driven: the bench then sees the wrong data); the controller driving DQ while the model drives read
// data is an error (bus contention).
//
// Functional, not a datasheet timing model. It checks the protocol rules the
// vendored controller relies on and reports violations as errors:
//   * mode register loaded before use; CL=2, BL=4 sequential, single writes
//   * ACTIVE only to an idle bank; READ/WRITE only to an active bank
//   * tRCD and tRFC honoured; REFRESH only with every bank precharged
//   * refresh commands are counted (timestamps) for the bench's rate check
// Datasheet recovery after auto-precharge (tRAS, tWR + tRP = tDAL) is only
// COUNTED (timing_warnings), not failed: the NA-1-proven controller issues
// the next ACTIVE/REFRESH after a fixed number of idle states, and the bench
// reports how that compares with -6 grade numbers (M2_IMPLEMENTATION s.6).
// Read data timing is calibrated to the capture edge the NA-1 hardware
// proves for this controller (docs/M2_IMPLEMENTATION.md section 6): word 0 of
// a CL=2 burst is driven from the SDRAM_CLK edge CL cycles after READ and
// held for one clock. Physical I/O timing is checked by TimeQuest, not here.
module sdr_sdram_model #(
    parameter real TCK_NS  = 10.334,
    parameter real TRCD_NS = 18.0,
    parameter real TRP_NS  = 18.0,
    parameter real TRFC_NS = 60.0,
    parameter real TRAS_NS = 42.0,
    parameter int  TWR_CK  = 2,
    parameter real TRRD_NS = 12.0,
    parameter real TRC_NS  = 60.0,
    parameter bit  STRICT  = 1
) (
    input  wire        clk,
    input  wire        cke,
    input  wire        csn, rasn, casn, wen,
    input  wire [1:0]  ba,
    input  wire [12:0] a,
    input  wire        dqml, dqmh,
    inout  wire [15:0] dq
);
    // Unwritten cells read as this value (the bench only checks written data).
    localparam [15:0] UNWRITTEN = 16'hA5C3;

    // static array (a full 32 MiB image fits ModelSim's 32-bit address space; an associative array does not)
    bit [15:0] mem [0:(1 << 24) - 1];
    initial for (int i = 0; i < (1 << 24); i++) mem[i] = UNWRITTEN;

    integer errors = 0;
    integer refreshes = 0;
    integer timing_warnings = 0;
    // shortfall histogram: [after write / after read][clocks short 1..3+]
    integer short_wr [4];
    integer short_rd [4];
    bit     last_wr [4];
    longint cycle = 0;
    longint refresh_cycle [$];

    bit        mode_set = 0;
    bit  [2:0] cl = 0;
    bit  [2:0] bl_code = 0;
    bit        active [4];
    bit [12:0] open_row [4];
    longint    t_active [4];
    longint    t_idle [4];      // cycle from which the bank may be activated again
    longint    t_rfc_end = 0;

    // Read pipeline: slot i holds the word to drive at cycle (issue + CL + i).
    reg [15:0] rd_word [0:15];
    reg        rd_valid [0:15];
    reg [15:0] dq_out = 16'h0;
    reg        dq_oe = 1'b0;
    assign dq = dq_oe ? dq_out : 16'hZZZZ;

    function automatic int cyc(input real ns);
        return $rtoi((ns + TCK_NS - 0.001) / TCK_NS);
    endfunction

    initial begin
        for (int i = 0; i < 4; i++) begin active[i] = 0; t_idle[i] = 0; t_active[i] = 0;
            short_wr[i] = 0; short_rd[i] = 0; last_wr[i] = 0; end
        for (int i = 0; i < 16; i++) begin rd_valid[i] = 0; rd_word[i] = 0; end
    end

    task automatic note_short(input int b);
        longint sh = t_idle[b] - cycle;
        int k = (sh >= 3) ? 3 : int'(sh);
        if (last_wr[b]) short_wr[k]++; else short_rd[k]++;
    endtask

    task automatic warn(input string s);
        if (STRICT) begin err(s); return; end
        timing_warnings++;
        if (timing_warnings <= 5) $display("SDRAM-MODEL timing note @cycle %0d: %s", cycle, s);
    endtask

    task automatic err(input string s);
        errors++;
        if (errors <= 20) $display("SDRAM-MODEL ERROR @cycle %0d: %s", cycle, s);
    endtask

    wire [3:0] cmd = {csn, rasn, casn, wen};
    reg  [1:0] dqm_d1 = 2'b00, dqm_d2 = 2'b00;    // DQM sampled 1 and 2 clocks ago (read DQM latency 2)
    longint    t_last_act = -100;
    longint    t_act_bank [4];
    initial for (int i = 0; i < 4; i++) t_act_bank[i] = -100;

    always @(posedge clk) begin
        cycle <= cycle + 1;
        dqm_d2 <= dqm_d1;
        dqm_d1 <= {dqmh, dqml};

        // bus contention: the controller drives DQ while we are driving read data
        if (dq_oe && (^dq === 1'bx)) err("DQ contention (controller drives DQ during read data)");

        // Drive the next read word (scheduled one clock at a time). Masked by DQM two clocks earlier.
        dq_oe  <= rd_valid[0] && !(STRICT && (dqm_d2 != 2'b00));
        if (STRICT && rd_valid[0] && (dqm_d2 != 2'b00)) err("read word masked by DQM (A12/A11 non-zero 2 clocks before)");
        dq_out <= rd_word[0];
        for (int i = 0; i < 15; i++) begin rd_valid[i] = rd_valid[i+1]; rd_word[i] = rd_word[i+1]; end
        rd_valid[15] = 0;

        if (cke && !csn) begin
            case (cmd[2:0])
            3'b111: ; // NOP
            3'b000: begin // LOAD MODE
                for (int b = 0; b < 4; b++) if (active[b]) err("LOAD MODE with an active bank");
                mode_set = 1;
                cl       = a[6:4];
                bl_code  = a[2:0];
                if (a[6:4] != 3'd2) err($sformatf("CAS latency %0d, expected 2", a[6:4]));
                if (a[2:0] != 3'b010) err("burst length is not 4");
                if (a[3] != 1'b0) err("burst type is not sequential");
                if (a[9] != 1'b1) err("write burst mode is not single-location");
            end
            3'b001: begin // AUTO REFRESH
                for (int b = 0; b < 4; b++) if (active[b]) err("REFRESH with an open bank");
                for (int b = 0; b < 4; b++) if (cycle < t_idle[b]) begin
                    note_short(b);
                    warn($sformatf("REFRESH %0d clk before bank %0d auto-precharge recovery after a %s",
                                   t_idle[b] - cycle, b, last_wr[b] ? "write" : "read"));
                end
                if (cycle < t_rfc_end) err("REFRESH inside tRFC");
                t_rfc_end = cycle + cyc(TRFC_NS);
                if (mode_set) begin refreshes++; refresh_cycle.push_back(cycle); end
            end
            3'b010: begin // PRECHARGE
                if (a[10]) for (int b = 0; b < 4; b++) begin
                    active[b] = 0; t_idle[b] = cycle + cyc(TRP_NS);
                end else begin
                    active[ba] = 0; t_idle[ba] = cycle + cyc(TRP_NS);
                end
            end
            3'b011: begin // ACTIVE
                if (!mode_set) err("ACTIVE before LOAD MODE");
                if (active[ba]) err($sformatf("ACTIVE to open bank %0d", ba));
                if (cycle < t_idle[ba]) begin
                    note_short(ba);
                    warn($sformatf("ACTIVE %0d clk before bank %0d auto-precharge recovery after a %s",
                                   t_idle[ba] - cycle, ba, last_wr[ba] ? "write" : "read"));
                end
                if (cycle < t_rfc_end) err("ACTIVE inside tRFC");
                if (cycle - t_last_act < cyc(TRRD_NS)) err("tRRD violated");
                if (cycle - t_act_bank[ba] < cyc(TRC_NS)) err($sformatf("tRC violated on bank %0d", ba));
                t_last_act = cycle;
                t_act_bank[ba] = cycle;
                active[ba]   = 1;
                open_row[ba] = a;
                t_active[ba] = cycle;
                t_idle[ba]   = 0;
            end
            3'b101, 3'b100: begin // READ / WRITE
                automatic bit        rd = (cmd[2:0] == 3'b101);
                automatic bit [8:0]  col = a[8:0];
                if (!active[ba]) err($sformatf("%s to idle bank %0d", rd ? "READ" : "WRITE", ba));
                else if (cycle - t_active[ba] < cyc(TRCD_NS)) err("tRCD violated");
                if (rd) begin
                    for (int i = 0; i < 4; i++) begin
                        automatic bit [8:0]  c = {col[8:2], col[1:0] + i[1:0]};   // sequential wrap in 4
                        automatic bit [23:0] k = {ba, open_row[ba], c};
                        rd_valid[cl - 1 + i] = 1;
                        rd_word [cl - 1 + i] = mem[k];
                    end
                end else begin
                    automatic bit [23:0] k = {ba, open_row[ba], col};
                    automatic bit [15:0] old = mem[k];
                    mem[k] = {dqmh ? old[15:8] : dq[15:8], dqml ? old[7:0] : dq[7:0]};
                end
                if (a[10]) begin // auto precharge: logically closed now, recovered later
                    automatic longint ready = t_active[ba] + cyc(TRAS_NS);
                    automatic longint endc  = cycle + (rd ? 4 : TWR_CK);
                    t_idle[ba] = ((ready > endc) ? ready : endc) + cyc(TRP_NS);
                    active[ba] = 0;
                    last_wr[ba] = !rd;
                end else err("READ/WRITE without auto-precharge (controller always uses A10=1)");
            end
            default: err($sformatf("unexpected command %b", cmd));
            endcase
        end
    end
endmodule
