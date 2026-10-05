// Namco NB-1 MiSTer core -- logical memory front end (M2).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// CLIENTS request/response ports above one physical SDRAM controller channel
// (ch1 of rtl/vendor/sdram.sv). Clients see logical (region, offset)
// addresses from nb1_mem_pkg and 32-bit big-endian data; they never see
// SDRAM rows, banks, bursts or the controller's pulse protocol.
//
// ---------------------------------------------------------------------------
// Client contract (docs/M2_IMPLEMENTATION.md section 4). For client i:
//
//  request   req_valid, req_region[3:0], req_offset[24:0], req_we,
//            req_wdata[31:0], req_be[3:0]
//            A request transfers on the edge where req_valid && req_ready.
//            Hold every request field stable while req_valid is high and
//            req_ready is low. req_ready may stay low for any number of
//            cycles (another client, refresh, controller start-up).
//  address   byte offset inside the region; offset[1:0] are ignored (the
//            access is to the aligned longword). Lanes are big-endian:
//            be[3]/wdata[31:24] = byte offset+0 ... be[0]/wdata[7:0] = +3.
//  response  rsp_valid is a one-cycle pulse, exactly one per accepted
//            request (reads and writes), in request order. rsp_rdata is the
//            aligned longword (reads; 0 for writes); rsp_err=1 means the
//            request was out of range (unknown region or offset beyond the
//            window): nothing was read or written, rsp_rdata=0.
//  line      (M3) rsp_line is the whole aligned 8-byte line that contains
//            the offset, big-endian: rsp_line[63:56] = byte (offset & ~7)+0
//            ... rsp_line[7:0] = +7. Valid with rsp_valid for reads (0 for
//            writes and errors). It is the controller's 4-word burst put in
//            address order, so a line fill costs one ordinary read.
//            rsp_rdata is always the longword at offset (= one half of the
//            line), so M2 clients are unaffected.
//  latency   NOT fixed. It depends on refresh, other clients and future
//            controllers. Clients must wait for rsp_valid, never count.
//  depth     a client keeps at most one request outstanding (accepted, not
//            yet answered). A client ignores rsp_valid when it has nothing
//            outstanding (for example after it reset itself mid-request).
//
// This lets a CPU wrapper stall on an outstanding request and account the
// stalled cycles for catch-up scheduling (standing rule SR-2).
// ---------------------------------------------------------------------------
//
// M2 arbitration: one logical transaction at a time, fixed priority (lowest
// index wins). Later milestones replace this block with a real multi-client
// scheduler behind the same client contract.
//
// M11 background hold (LOW_HOLD, default 0 = M2 behaviour): the lowest-
// priority client (index CLIENTS-1) is not granted in the LOW_HOLD cycles
// after a response to any other client. A streaming client re-requests two
// cycles after its response (response seen, next request registered), so
// without the hold a waiting background client takes every other slot and
// halves a higher-priority client's throughput -- on hardware this made the
// C123 tile renderer (client 4) miss line deadlines behind the C355 sprite
// renderer (client 5) (docs/M11_IMPLEMENTATION.md section 9).
//
// Reset policy: this block is reset only by `init` (PLL loss of lock, the
// same signal that re-initialises the SDRAM). OSD/user resets and ROM
// downloads never reset it, so a reset cannot abort an SDRAM write halfway
// or desynchronise the controller.
//
// Physical side (vendored controller, verified by NA-1 and sim/m2_mem_tb):
// phy_req is a one-cycle pulse, OR-latched by the controller; the next pulse
// is only issued after phy_ready of the previous operation. Reads are 4-word
// bursts; phy_dout[63:48] is written one edge after phy_ready, so read data
// is captured one cycle after phy_ready (all four words valid). Writes are
// single 16-bit words with byte masks, so a 32-bit write becomes up to two
// physical writes (halves with no enabled byte are skipped).
//
// M23 pipelined reads (PIPE = 1, production): a read is sent to the controller
// as soon as the previous operation has been TAKEN (phy_taken, NB-1 M23 delta
// of rtl/vendor/sdram.sv), so the next ACTIVE follows the previous burst with
// no front-end turnaround (~8 instead of ~14 clk_sys per read when a client
// keeps two reads queued). At most two reads are at the controller (one in
// service, one latched); responses leave in issue order (a queue of owners),
// one cycle after phy_ready as before. The client contract gains one rule:
// with PIPE = 1 a client MAY keep up to two requests outstanding; their
// responses arrive in request order. Clients that keep one are unaffected.
// Writes and out-of-range requests keep the M2 semantics: a write waits until
// no read is in flight, then runs to completion before anything else is
// issued; an out-of-range request is answered in order through the queue.
// PIPE = 0 is the M2-M22 front end, unchanged.
module nb1_memory #(parameter int CLIENTS = 2, parameter int LOW_HOLD = 0,
                    parameter bit PIPE = 1'b0) (   // M23: 1 = pipelined reads (production)
    input  wire                    clk_sys,
    input  wire                    init,

    input  wire [CLIENTS-1:0]      req_valid,
    output wire [CLIENTS-1:0]      req_ready,
    input  wire [CLIENTS*4-1:0]    req_region,
    input  wire [CLIENTS*25-1:0]   req_offset,
    input  wire [CLIENTS-1:0]      req_we,
    input  wire [CLIENTS*32-1:0]   req_wdata,
    input  wire [CLIENTS*4-1:0]    req_be,
    output reg  [CLIENTS-1:0]      rsp_valid = '0,
    output reg  [31:0]             rsp_rdata = '0,   // shared, valid with rsp_valid[i]
    output reg  [63:0]             rsp_line  = '0,   // shared, valid with rsp_valid[i] (M3)
    output reg                     rsp_err   = 1'b0,

    // ch1 of rtl/vendor/sdram.sv
    output reg                     phy_req  = 1'b0,
    output reg                     phy_rnw  = 1'b1,
    output reg  [26:1]             phy_addr = '0,
    output reg  [15:0]             phy_din  = '0,
    output reg  [1:0]              phy_be   = 2'b11,
    input  wire                    phy_ready,
    input  wire                    phy_taken,     // M23: the controller took the latched request (PIPE = 1)
    input  wire [63:0]             phy_dout,

    output wire                    busy
);
    import nb1_mem_pkg::*;

    localparam int OW = (CLIENTS > 1) ? $clog2(CLIENTS) : 1;

    // Background hold: cycles left in which client CLIENTS-1 may not be granted.
    reg [1:0]         hold = '0;
    wire [CLIENTS-1:0] req_elig = req_valid & ~((hold != 2'd0) ? (CLIENTS'(1) << (CLIENTS - 1)) : '0);

    // Fixed-priority grant among eligible clients, only in S_IDLE.
    reg          grant;
    reg [OW-1:0] grant_idx;
    always_comb begin
        grant = 1'b0;
        grant_idx = '0;
        for (int i = CLIENTS - 1; i >= 0; i--) begin
            if (req_elig[i]) begin
                grant = 1'b1;
                grant_idx = i[OW-1:0];
            end
        end
    end

    // Granted client's fields as a constant-index mux (no variable part-select).
    reg [3:0]  g_region;
    reg [24:0] g_offset;
    reg        g_we;
    reg [31:0] g_wdata;
    reg [3:0]  g_be;
    always_comb begin
        g_region = '0; g_offset = '0; g_we = 1'b0; g_wdata = '0; g_be = '0;
        for (int j = 0; j < CLIENTS; j++) begin
            if (grant_idx == j) begin
                g_region = req_region[j*4 +: 4];
                g_offset = req_offset[j*25 +: 25];
                g_we     = req_we[j];
                g_wdata  = req_wdata[j*32 +: 32];
                g_be     = req_be[j*4 +: 4];
            end
        end
    end

    genvar g;
    generate if (!PIPE) begin : g_serial
    localparam [2:0] S_IDLE    = 3'd0,
                     S_DECODE  = 3'd1,
                     S_WAIT_RD = 3'd2,
                     S_CAPTURE = 3'd3,
                     S_WAIT_WH = 3'd4,
                     S_WAIT_WL = 3'd5,
                     S_RESP    = 3'd6;

    reg [2:0]    state = S_IDLE;
    reg [OW-1:0] owner = '0;
    reg [3:0]    l_region = '0;
    reg [24:0]   l_offset = '0;
    reg          l_we = 1'b0;
    reg [31:0]   l_wdata = '0;
    reg [3:0]    l_be = '0;
    reg [24:0]   l_phys = '0;
    reg          l_err = 1'b0;
    reg [31:0]   l_rdata = '0;
    reg [63:0]   l_line  = '0;

    assign busy = (state != S_IDLE);

    for (g = 0; g < CLIENTS; g++) begin : ready_gen
        assign req_ready[g] = (state == S_IDLE) && !init && grant && (grant_idx == g);
    end

    phys_t xlat;
    always_comb xlat = region_to_phys(l_region, {l_offset[24:2], 2'b00});

    task automatic issue(input logic rnw, input logic [24:0] byte_addr,
                         input logic [15:0] din, input logic [1:0] be);
        phy_req  <= 1'b1;
        phy_rnw  <= rnw;
        phy_addr <= {2'b00, byte_addr[24:1]};
        phy_din  <= din;
        phy_be   <= be;
    endtask

    always @(posedge clk_sys) begin
        phy_req   <= 1'b0;
        rsp_valid <= '0;
        if (hold != 2'd0) hold <= hold - 2'd1;

        if (init) begin
            state <= S_IDLE;
        end else begin
            case (state)
            S_IDLE: if (grant) begin
                owner    <= grant_idx;
                l_region <= g_region;
                l_offset <= g_offset;
                l_we     <= g_we;
                l_wdata  <= g_wdata;
                l_be     <= g_be;
                state    <= S_DECODE;
            end

            S_DECODE: begin
                l_phys  <= xlat.phys;
                l_err   <= xlat.oob;
                l_rdata <= 32'd0;
                l_line  <= 64'd0;
                if (xlat.oob) begin
                    state <= S_RESP;
                end else if (!l_we) begin
                    issue(1'b1, xlat.phys, 16'd0, 2'b11);
                    state <= S_WAIT_RD;
                end else if (l_be[3:2] != 2'b00) begin
                    issue(1'b0, xlat.phys, l_wdata[31:16], l_be[3:2]);
                    state <= S_WAIT_WH;
                end else if (l_be[1:0] != 2'b00) begin
                    issue(1'b0, xlat.phys + 25'd2, l_wdata[15:0], l_be[1:0]);
                    state <= S_WAIT_WL;
                end else begin
                    state <= S_RESP;             // no byte enabled: nothing to do
                end
            end

            S_WAIT_RD: if (phy_ready) state <= S_CAPTURE;

            S_CAPTURE: begin
                // Burst word 0 = bytes A,A+1; word 1 = bytes A+2,A+3.
                l_rdata <= {phy_dout[15:0], phy_dout[31:16]};
                // The burst wraps inside the aligned 4-word block (sequential
                // burst, BL4): it starts at word A[2:1] = {offset[2],0}.
                // Rotate it back into address order for the line response.
                l_line  <= l_offset[2] ? {phy_dout[47:32], phy_dout[63:48], phy_dout[15:0], phy_dout[31:16]}
                                       : {phy_dout[15:0],  phy_dout[31:16], phy_dout[47:32], phy_dout[63:48]};
                state   <= S_RESP;
            end

            S_WAIT_WH: if (phy_ready) begin
                if (l_be[1:0] != 2'b00) begin
                    issue(1'b0, l_phys + 25'd2, l_wdata[15:0], l_be[1:0]);
                    state <= S_WAIT_WL;
                end else begin
                    state <= S_RESP;
                end
            end

            S_WAIT_WL: if (phy_ready) state <= S_RESP;

            S_RESP: begin
                rsp_valid[owner] <= 1'b1;
                rsp_rdata        <= l_rdata;
                rsp_line         <= l_line;
                rsp_err          <= l_err;
                state            <= S_IDLE;
                if (LOW_HOLD != 0 && owner != OW'(CLIENTS - 1)) hold <= 2'(LOW_HOLD);
            end

            default: state <= S_IDLE;
            endcase
        end
    end
    end else begin : g_pipe
    // ================================================================ M23 pipelined
    phys_t dx;
    reg          d_valid = 1'b0;                 // decode stage: one accepted request
    reg [OW-1:0] d_owner = '0;
    reg [3:0]    d_region = '0;
    reg [24:0]   d_offset = '0;
    reg          d_we = 1'b0;
    reg [31:0]   d_wdata = '0;
    reg [3:0]    d_be = '0;
    always_comb dx = region_to_phys(d_region, {d_offset[24:2], 2'b00});

    // response queue in issue order: {owner, nophy (answered without the controller), err, offset[2]}
    reg [OW+2:0] q [4];
    reg [1:0]    q_rd = '0, q_wr = '0;
    reg [2:0]    q_cnt = '0;
    wire [OW+2:0] q_head = q[q_rd];
    reg          take_pend = 1'b0;               // a request pulsed, not yet taken
    reg [1:0]    phy_rd = '0;                    // reads at the controller (ready not yet seen)
    reg          rdy_q = 1'b0;                   // a read's phy_ready was seen last cycle
    // serial write (M2 semantics): 0 none, 1 high half at the controller, 2 low half
    reg [1:0]    w_st = '0;
    reg [OW-1:0] w_owner = '0;
    reg [24:0]   w_phys = '0;
    reg [15:0]   w_low = '0;
    reg [1:0]    w_lbe = '0;

    wire acc_ok = !d_valid;
    for (g = 0; g < CLIENTS; g++) begin : ready_gen
        assign req_ready[g] = acc_ok && !init && grant && (grant_idx == g);
    end
    assign busy = d_valid || (q_cnt != 3'd0) || (w_st != 2'd0) || take_pend;

    wire can_read  = !take_pend && (phy_rd != 2'd2) && (q_cnt != 3'd4) && (w_st == 2'd0);
    wire can_write = !take_pend && (phy_rd == 2'd0) && (q_cnt == 3'd0) && (w_st == 2'd0) && !rdy_q;

    always @(posedge clk_sys) begin
        logic          push, pop, issue_rd;
        logic [OW+2:0] pv;
        push = 1'b0; pop = 1'b0; pv = '0; issue_rd = 1'b0;
        phy_req   <= 1'b0;
        rsp_valid <= '0;
        rdy_q     <= 1'b0;
        if (hold != 2'd0) hold <= hold - 2'd1;
        if (phy_taken) take_pend <= 1'b0;

        if (init) begin
            d_valid <= 1'b0; q_rd <= '0; q_wr <= '0; q_cnt <= '0;
            take_pend <= 1'b0; phy_rd <= '0; w_st <= '0;
        end else begin
            // ---- accept
            if (acc_ok && grant) begin
                d_valid  <= 1'b1;
                d_owner  <= grant_idx;
                d_region <= g_region;
                d_offset <= g_offset;
                d_we     <= g_we;
                d_wdata  <= g_wdata;
                d_be     <= g_be;
            end

            // ---- decode / issue
            if (d_valid) begin
                if (dx.oob) begin
                    if (q_cnt != 3'd4 && w_st == 2'd0) begin   // in order behind reads, never beside a write
                        push = 1'b1; pv = {d_owner, 1'b1, 1'b1, 1'b0};
                        d_valid <= 1'b0;
                    end
                end else if (!d_we) begin
                    if (can_read) begin
                        phy_req  <= 1'b1; phy_rnw <= 1'b1; phy_addr <= {2'b00, dx.phys[24:1]};
                        phy_din  <= 16'd0; phy_be <= 2'b11; take_pend <= 1'b1;
                        push = 1'b1; pv = {d_owner, 1'b0, 1'b0, d_offset[2]}; issue_rd = 1'b1;
                        d_valid <= 1'b0;
                    end
                end else if (can_write) begin
                    d_valid <= 1'b0;
                    w_owner <= d_owner;
                    w_phys  <= dx.phys;
                    w_low   <= d_wdata[15:0];
                    w_lbe   <= d_be[1:0];
                    if (d_be[3:2] != 2'b00) begin
                        phy_req <= 1'b1; phy_rnw <= 1'b0; phy_addr <= {2'b00, dx.phys[24:1]};
                        phy_din <= d_wdata[31:16]; phy_be <= d_be[3:2]; take_pend <= 1'b1;
                        w_st <= 2'd1;
                    end else if (d_be[1:0] != 2'b00) begin
                        phy_req <= 1'b1; phy_rnw <= 1'b0; phy_addr <= {2'b00, dx.phys[24:1]} + 25'd1;
                        phy_din <= d_wdata[15:0]; phy_be <= d_be[1:0]; take_pend <= 1'b1;
                        w_st <= 2'd2;
                    end else begin
                        push = 1'b1; pv = {d_owner, 1'b1, 1'b0, 1'b0};   // no byte enabled: nothing to do
                    end
                end
            end

            // ---- controller completions
            if (phy_ready) begin
                if (w_st == 2'd1 && w_lbe != 2'b00) begin
                    phy_req <= 1'b1; phy_rnw <= 1'b0; phy_addr <= {2'b00, w_phys[24:1]} + 25'd1;
                    phy_din <= w_low; phy_be <= w_lbe; take_pend <= 1'b1;
                    w_st <= 2'd2;
                end else if (w_st != 2'd0) begin
                    w_st <= 2'd0;
                    rsp_valid[w_owner] <= 1'b1;
                    rsp_rdata <= 32'd0; rsp_line <= 64'd0; rsp_err <= 1'b0;
                    if (LOW_HOLD != 0 && w_owner != OW'(CLIENTS - 1)) hold <= 2'(LOW_HOLD);
                end else begin
                    rdy_q <= 1'b1;                    // a read: all four words valid one edge later
                end
            end

            // ---- responses, in issue order (at most one per cycle)
            if (rdy_q) begin
                // Burst word 0 = bytes A,A+1; word 1 = bytes A+2,A+3; the burst wraps in the
                // aligned 4-word block, starting at word {offset[2],0} (as the M2 path).
                rsp_valid[q_head[OW+2:3]] <= 1'b1;
                rsp_rdata <= {phy_dout[15:0], phy_dout[31:16]};
                rsp_line  <= q_head[0] ? {phy_dout[47:32], phy_dout[63:48], phy_dout[15:0], phy_dout[31:16]}
                                       : {phy_dout[15:0],  phy_dout[31:16], phy_dout[47:32], phy_dout[63:48]};
                rsp_err   <= 1'b0;
                pop = 1'b1;
                if (LOW_HOLD != 0 && q_head[OW+2:3] != OW'(CLIENTS - 1)) hold <= 2'(LOW_HOLD);
            end else if (q_cnt != 3'd0 && q_head[2]) begin
                rsp_valid[q_head[OW+2:3]] <= 1'b1;
                rsp_rdata <= 32'd0; rsp_line <= 64'd0; rsp_err <= q_head[1];
                pop = 1'b1;
                if (LOW_HOLD != 0 && q_head[OW+2:3] != OW'(CLIENTS - 1)) hold <= 2'(LOW_HOLD);
            end

            // ---- queue and read-count bookkeeping
            if (push) begin q[q_wr] <= pv; q_wr <= q_wr + 2'd1; end
            if (pop) q_rd <= q_rd + 2'd1;
            q_cnt  <= q_cnt + (push ? 3'd1 : 3'd0) - (pop ? 3'd1 : 3'd0);
            phy_rd <= phy_rd + (issue_rd ? 2'd1 : 2'd0) - ((phy_ready && w_st == 2'd0) ? 2'd1 : 2'd0);
        end
    end
    end endgenerate
endmodule
