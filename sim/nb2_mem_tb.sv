// Namco NB-2 MiSTer core -- bench for nb2_sdram + nb2_memory on the strict SDRAM model.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// 1. Startup, then every client runs a random mix of 32-bit writes (random byte enables) and reads in its own
//    address partition (spread over every SDRAM region and all four banks), up to OUT_MAX requests in flight,
//    with random request gaps. Every read is checked against a shadow memory (longword and 8-byte line), the
//    responses against each client's request order, and out-of-range requests (OBJ region, offsets beyond a
//    window) for rsp_err. The model fails on any protocol, recovery, DQM-masking or contention violation.
// 2. Throughput: two clients stream random 8-byte reads (the ROZ/tile pattern) for 8 video lines; the bench
//    reports reads per 6,144-clock line.
`timescale 1ns/1ps
module nb2_mem_tb;
    import nb2_mem_pkg::*;
    localparam int CL = 4, OUT_MAX = 2;
    localparam int CW = 2, TW = CW + 2;

    reg clk = 1'b0;
    always #5.167 clk = ~clk;
    longint cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;
    integer errors = 0;
    task automatic fail(input string s);
        errors++;
        if (errors <= 30) $display("ERROR @%0d: %s", cyc, s);
    endtask

    reg init = 1'b1;

    wire [15:0] DQ;
    wire [12:0] A;
    wire [1:0]  BA;
    wire        DQML, DQMH, nCS, nWE, nRAS, nCAS, CKE, SCLK;
    wire        sd_valid, sd_ready, sd_we, sd_rsp_valid, sd_rsp_write, sd_up;
    wire [24:1] sd_addr;
    wire [15:0] sd_wdata;
    wire [1:0]  sd_be;
    wire [TW-1:0] sd_tag, sd_rsp_tag;
    wire [63:0] sd_rsp_line;

    nb2_sdram #(.TW(TW)) sdram (
        .clk(clk), .init(init),
        .req_valid(sd_valid), .req_ready(sd_ready), .req_we(sd_we), .req_addr(sd_addr), .req_wdata(sd_wdata),
        .req_be(sd_be), .req_tag(sd_tag),
        .rsp_valid(sd_rsp_valid), .rsp_write(sd_rsp_write), .rsp_tag(sd_rsp_tag), .rsp_line(sd_rsp_line),
        .ready(sd_up),
        .SDRAM_DQ(DQ), .SDRAM_A(A), .SDRAM_DQML(DQML), .SDRAM_DQMH(DQMH), .SDRAM_BA(BA), .SDRAM_nCS(nCS),
        .SDRAM_nWE(nWE), .SDRAM_nRAS(nRAS), .SDRAM_nCAS(nCAS), .SDRAM_CKE(CKE), .SDRAM_CLK(SCLK));

    sdr_sdram_model model (.clk(SCLK), .cke(CKE), .csn(nCS), .rasn(nRAS), .casn(nCAS), .wen(nWE), .ba(BA), .a(A),
                           .dqml(DQML), .dqmh(DQMH), .dq(DQ));

    reg  [CL-1:0]    req_valid = '0;
    wire [CL-1:0]    req_ready;
    reg  [CL*4-1:0]  req_region = '0;
    reg  [CL*26-1:0] req_offset = '0;
    reg  [CL-1:0]    req_we = '0;
    reg  [CL*32-1:0] req_wdata = '0;
    reg  [CL*4-1:0]  req_be = '0;
    wire [CL-1:0]    rsp_valid;
    wire [31:0]      rsp_rdata;
    wire [63:0]      rsp_line;
    wire             rsp_err;

    nb2_memory #(.CLIENTS(CL), .OUT_MAX(OUT_MAX)) mem (
        .clk_sys(clk), .init(init),
        .req_valid(req_valid), .req_ready(req_ready), .req_region(req_region), .req_offset(req_offset),
        .req_we(req_we), .req_wdata(req_wdata), .req_be(req_be),
        .rsp_valid(rsp_valid), .rsp_rdata(rsp_rdata), .rsp_line(rsp_line), .rsp_err(rsp_err),
        .sd_valid(sd_valid), .sd_ready(sd_ready), .sd_we(sd_we), .sd_addr(sd_addr), .sd_wdata(sd_wdata),
        .sd_be(sd_be), .sd_tag(sd_tag), .sd_rsp_valid(sd_rsp_valid), .sd_rsp_write(sd_rsp_write),
        .sd_rsp_tag(sd_rsp_tag), .sd_rsp_line(sd_rsp_line), .busy_clients());

    // ------------------------------------------------------------------ shadow memory (physical bytes)
    byte unsigned shadow [int];
    function automatic byte unsigned sh(input int a);
        // unwritten words read the model's UNWRITTEN pattern $A5C3 (high byte = even address)
        if (shadow.exists(a)) return shadow[a];
        return a[0] ? 8'hC3 : 8'hA5;
    endfunction

    // expected-response queues per client
    typedef struct { bit we; bit err; int phys; logic [31:0] xl; logic [63:0] xline; } exp_t;
    exp_t q [CL][$];

    // ------------------------------------------------------------------ client stimulus
    localparam logic [3:0] REGS [9] = '{REG_ROZ, REG_CHR, REG_VOICE, REG_SHAPE, REG_ROZMASK, REG_PROG,
                                        REG_DATA, REG_C75DATA, REG_WRAM};
    int unsigned seed = 1;
    function automatic int unsigned rnd();
        seed = seed * 1664525 + 1013904223;
        return seed;
    endfunction

    bit run = 0, stream = 0;
    bit [CL-1:0] only = '1;          // clients allowed to issue in the mixed phase
    int issued [CL], answered [CL];
    // partition: client c owns 8-byte blocks whose index % CL == c, inside a 16 KiB slice of each region
    task automatic make_req(input int c, output logic [3:0] rg, output logic [25:0] off, output bit we,
                            output logic [31:0] wd, output logic [3:0] be, output bit err);
        int r = rnd() % 100;
        err = 0;
        if (r < 3) begin                                         // out of range
            err = 1; we = 0;
            if (rnd() % 2) begin rg = REG_OBJ; off = 26'(rnd() % 32'h1400000); end
            else begin rg = REG_C75BIOS; off = 26'h4000 + 26'(rnd() % 32'h1000); end
            wd = '0; be = 4'hF;
            return;
        end
        rg = REGS[rnd() % 9];
        off = 26'(((rnd() % 2048) * CL + c) * 8 + (rnd() % 2) * 4);
        if (rg == REG_PROG || rg == REG_DATA) off = off & 26'h00FFFFF;
        we = stream ? 0 : (rnd() % 3 == 0);
        wd = rnd();
        be = we ? 4'(rnd() % 16) : 4'hF;
    endtask

    for (genvar c = 0; c < CL; c++) begin : cli
        int inflight = 0;
        always @(posedge clk) begin
            logic [3:0] rg; logic [25:0] off; bit we, err; logic [31:0] wd; logic [3:0] be;
            if (req_valid[c] && req_ready[c]) begin
                // record the expectation and apply writes to the shadow at acceptance
                phys_t px;
                exp_t e;
                px = region_to_phys(req_region[c*4 +: 4], {req_offset[c*26+2 +: 24], 2'b00});
                e.we = req_we[c]; e.err = px.oob || (req_region[c*4 +: 4] == REG_OBJ); e.phys = px.phys;
                // expected read data = memory at acceptance (the controller works in acceptance order)
                for (int b = 0; b < 4; b++) e.xl[31 - 8*b -: 8] = sh(px.phys + b);
                for (int b = 0; b < 8; b++) e.xline[63 - 8*b -: 8] = sh((px.phys & ~7) + b);
                q[c].push_back(e);
                if (req_we[c] && !e.err)
                    for (int b = 0; b < 4; b++) if (req_be[c*4 + 3 - b]) shadow[px.phys + b] = req_wdata[c*32 + 31 - 8*b -: 8];
                req_valid[c] <= 1'b0;
                issued[c]++;
            end
            if ((run || stream) && (!req_valid[c] || req_ready[c]) && (stream || (rnd() % 100) < 25 + 17 * c) &&
                ((issued[c] + (req_valid[c] && req_ready[c] ? 1 : 0)) - answered[c] < OUT_MAX + 1)) begin
                if ((!stream && only[c]) || (stream && c < 2)) begin
                    make_req(c, rg, off, we, wd, be, err);
                    req_valid[c] <= 1'b1;
                    req_region[c*4 +: 4] <= rg;
                    req_offset[c*26 +: 26] <= off;
                    req_we[c] <= we;
                    req_wdata[c*32 +: 32] <= wd;
                    req_be[c*4 +: 4] <= be;
                end
            end
            if (rsp_valid[c]) begin
                exp_t e;
                answered[c]++;
                if (q[c].size() == 0) fail($sformatf("client %0d: response with nothing outstanding", c));
                else begin
                    e = q[c].pop_front();
                    if (rsp_err != e.err) fail($sformatf("client %0d: rsp_err %0d, expected %0d (phys %h)", c, rsp_err, e.err, e.phys));
                    else if (!e.we && !e.err) begin
                        if (rsp_rdata !== e.xl) fail($sformatf("client %0d read %h: %h, expected %h", c, e.phys, rsp_rdata, e.xl));
                        if (rsp_line !== e.xline) fail($sformatf("client %0d line %h: %h, expected %h", c, e.phys & ~7, rsp_line, e.xline));
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------ throughput
    longint t0, n0;
    initial begin
        int lines;
        for (int c = 0; c < CL; c++) begin issued[c] = 0; answered[c] = 0; end
        repeat (20) @(posedge clk);
        init = 1'b0;
        wait (sd_up);
        repeat (10) @(posedge clk);
        run = 1;
        repeat (400000) @(posedge clk);
        only = 4'b1100;                  // the low-priority clients on their own
        repeat (200000) @(posedge clk);
        run = 0;
        wait (issued[0] == answered[0] && issued[1] == answered[1] && issued[2] == answered[2] && issued[3] == answered[3]);
        repeat (50) @(posedge clk);
        $display("mixed phase: %0d/%0d/%0d/%0d requests answered", answered[0], answered[1], answered[2], answered[3]);
        // throughput: two streaming readers
        stream = 1;
        repeat (2000) @(posedge clk);
        t0 = cyc; n0 = answered[0] + answered[1];
        repeat (6144 * 8) @(posedge clk);
        $display("THROUGHPUT: %0d reads per 6144-clock line (2 clients x %0d in flight)",
                 (answered[0] + answered[1] - n0) / 8, OUT_MAX);
        stream = 0;
        wait (issued[0] == answered[0] && issued[1] == answered[1]);
        repeat (50) @(posedge clk);
        $display("model: %0d errors, %0d refreshes; bench: %0d errors", model.errors, model.refreshes, errors);
        if (errors == 0 && model.errors == 0 && answered[0] > 1000) $display("PASS nb2_mem_tb");
        else $display("FAIL nb2_mem_tb");
        $finish;
    end
    initial begin
        #200ms;
        $display("FAIL nb2_mem_tb: timeout");
        $finish;
    end
endmodule
