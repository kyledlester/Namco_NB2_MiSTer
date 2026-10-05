// Namco NB-2 MiSTer core -- SDRAM front end: CLIENTS logical ports over nb2_sdram.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Same client contract as the NB-1 core's nb1_memory (docs/PROVENANCE.md), so the NB-1 clients (program cache,
// C75, C352, tile and sprite renderers, loader, checker) connect unchanged:
//   request   req_valid/req_ready handshake; region, byte offset (offset[1:0] ignored), we, wdata (big-endian
//             lanes: be[3]/wdata[31:24] = byte offset+0), be.
//   response  rsp_valid[i] one-cycle pulse per accepted request, in that client's request order; rsp_rdata = the
//             aligned longword, rsp_line = the aligned 8-byte line {byte +0 .. +7} (reads); rsp_err = out of range
//             (nothing done, data 0).
//   depth     a client may keep up to OUT_MAX requests outstanding (NB-1 M23's PIPE contract: two); responses
//             to one client come back in its request order.
// Differences from nb1_memory: the NB-2 map (nb2_mem_pkg: SDRAM regions only; OBJ is the DDR3 store and is
// answered rsp_err here), the pipelined controller (several accesses in flight across clients), and 26-bit
// offsets. Arbitration: fixed priority, lowest index first, one request granted per free issue slot.
//
// Byte order in SDRAM: word w holds byte 2w in [15:8] and byte 2w+1 in [7:0] (as NB-1), so a line
// {word0, word1, word2, word3} is already the big-endian 8-byte line.
module nb2_memory #(
    parameter int CLIENTS = 6,
    parameter int OUT_MAX = 2,
    parameter int CW = (CLIENTS > 1) ? $clog2(CLIENTS) : 1,
    parameter int TW = CW + 2                    // nb2_sdram tag: {client, offset[2], last word of the request}
) (
    input  wire                    clk_sys,
    input  wire                    init,

    input  wire [CLIENTS-1:0]      req_valid,
    output wire [CLIENTS-1:0]      req_ready,
    input  wire [CLIENTS*4-1:0]    req_region,
    input  wire [CLIENTS*26-1:0]   req_offset,
    input  wire [CLIENTS-1:0]      req_we,
    input  wire [CLIENTS*32-1:0]   req_wdata,
    input  wire [CLIENTS*4-1:0]    req_be,
    output reg  [CLIENTS-1:0]      rsp_valid = '0,
    output reg  [31:0]             rsp_rdata = '0,
    output reg  [63:0]             rsp_line  = '0,
    output reg                     rsp_err   = 1'b0,

    // nb2_sdram request/response port
    output wire                    sd_valid,
    input  wire                    sd_ready,
    output wire                    sd_we,
    output wire [24:1]             sd_addr,
    output wire [15:0]             sd_wdata,
    output wire [1:0]              sd_be,
    output wire [TW-1:0]           sd_tag,
    input  wire                    sd_rsp_valid,
    input  wire                    sd_rsp_write,
    input  wire [TW-1:0]           sd_rsp_tag,
    input  wire [63:0]             sd_rsp_line,

    output wire [7:0]              busy_clients
);
    import nb2_mem_pkg::*;

    // per-client outstanding count (accepted, not answered)
    reg [2:0] outst [0:CLIENTS-1];
    integer k;
    initial for (k = 0; k < CLIENTS; k++) outst[k] = '0;

    // ---------------------------------------------------------------- hold register (one granted request)
    reg          h_v = 1'b0;
    reg [CW-1:0] h_cl = '0;
    reg          h_we = 1'b0, h_err = 1'b0, h_off2 = 1'b0;
    reg [3:0]    h_region = '0;
    reg [25:0]   h_off = '0;
    reg [31:0]   h_wd = '0;
    reg [3:0]    h_be = '0;
    reg          h_half = 1'b0;                   // write: 0 = high word next, 1 = low word next

    // grant: lowest index with a request and room (outstanding < OUT_MAX); an out-of-range request only when
    // that client has nothing outstanding, so its error answer cannot overtake earlier answers
    reg          grant;
    reg [CW-1:0] gi;
    reg [CLIENTS-1:0] oob_c;
    always_comb begin
        for (int j = 0; j < CLIENTS; j++) begin
            phys_t px;
            px = region_to_phys(req_region[j*4 +: 4], {req_offset[j*26+2 +: 24], 2'b00});
            oob_c[j] = px.oob || (req_region[j*4 +: 4] == REG_OBJ);
        end
        grant = 1'b0; gi = '0;
        for (int j = CLIENTS - 1; j >= 0; j--)
            if (req_valid[j] && (outst[j] < 3'(OUT_MAX)) && (!oob_c[j] || outst[j] == 3'd0)) begin
                grant = 1'b1; gi = j[CW-1:0];
            end
    end
    reg [3:0]  g_region; reg [25:0] g_offset; reg g_we; reg [31:0] g_wdata; reg [3:0] g_be;
    always_comb begin
        g_region = '0; g_offset = '0; g_we = 1'b0; g_wdata = '0; g_be = '0;
        for (int j = 0; j < CLIENTS; j++) if (gi == j) begin
            g_region = req_region[j*4 +: 4]; g_offset = req_offset[j*26 +: 26]; g_we = req_we[j];
            g_wdata = req_wdata[j*32 +: 32]; g_be = req_be[j*4 +: 4];
        end
    end
    wire g_oob = oob_c[gi];
    // timing: the region base is added after the hold register (not behind the grant mux)
    phys_t hx;
    always_comb hx = region_to_phys(h_region, h_off);
    wire [25:0] h_phys = hx.phys;

    // the hold register is free this cycle if empty, or its last controller request is taken now
    wire h_wlast    = !h_we || h_half || (h_be[3:2] == 2'b00) || (h_be[1:0] == 2'b00);   // this word is the last
    wire h_last_now = h_v && !h_err && sd_ready && h_wlast;
    wire ctl_rsp    = sd_rsp_valid && sd_rsp_tag[0];
    wire err_rsp    = h_v && h_err && !ctl_rsp;
    wire h_free = !h_v || h_last_now || err_rsp;
    wire accept = grant && h_free && !init;
    genvar g;
    generate for (g = 0; g < CLIENTS; g = g + 1) begin : rdy
        assign req_ready[g] = accept && (gi == g);
    end endgenerate

    // controller request from the hold register
    wire h_word_lo = h_we && (h_half || h_be[3:2] == 2'b00);
    assign sd_valid = h_v && !h_err && !init;
    assign sd_we    = h_we;
    assign sd_addr  = h_we ? {h_phys[24:2], h_word_lo} : {h_phys[24:3], 2'b00};   // reads: the 8-byte line
    assign sd_wdata = h_word_lo ? h_wd[15:0] : h_wd[31:16];
    assign sd_be    = h_word_lo ? h_be[1:0] : h_be[3:2];
    assign sd_tag   = {h_cl, h_off2, h_wlast};

    wire [CW-1:0] r_cl = sd_rsp_tag[TW-1:2];
    always @(posedge clk_sys) begin
        rsp_valid <= '0;
        // outstanding bookkeeping happens with the response below
        if (init) begin
            h_v <= 1'b0;
            for (k = 0; k < CLIENTS; k++) outst[k] <= '0;
        end else begin
            // ---- hold register
            if (h_v && !h_err && sd_ready) begin
                if (!h_wlast) h_half <= 1'b1;                    // high word taken, low word next
                else h_v <= 1'b0;
            end
            if (err_rsp) h_v <= 1'b0;
            if (accept) begin
                h_v    <= 1'b1;
                h_cl   <= gi;
                h_we   <= g_we && (g_be != 4'b0000);
                h_err  <= g_oob;
                h_off2 <= g_offset[2];
                h_region <= g_region;
                h_off  <= {g_offset[25:2], 2'b00};
                h_wd   <= g_wdata;
                h_be   <= g_be;
                h_half <= 1'b0;
            end
            // ---- responses: controller (in issue order) or an out-of-range / empty write (from the hold reg)
            if (ctl_rsp) begin
                rsp_valid[r_cl] <= 1'b1;
                rsp_line  <= sd_rsp_write ? 64'd0 : sd_rsp_line;
                rsp_rdata <= sd_rsp_write ? 32'd0 : (sd_rsp_tag[1] ? sd_rsp_line[31:0] : sd_rsp_line[63:32]);
                rsp_err   <= 1'b0;
            end else if (err_rsp) begin
                rsp_valid[h_cl] <= 1'b1;
                rsp_line <= '0; rsp_rdata <= '0; rsp_err <= 1'b1;
            end
            for (k = 0; k < CLIENTS; k++)
                outst[k] <= outst[k] + ((accept && gi == k) ? 3'd1 : 3'd0)
                          - (((ctl_rsp && r_cl == k) || (err_rsp && h_cl == k)) ? 3'd1 : 3'd0);
        end
    end

    generate for (g = 0; g < 8; g = g + 1) begin : bz
        if (g < CLIENTS) begin : yes assign busy_clients[g] = (outst[g] != 3'd0); end
        else begin : no assign busy_clients[g] = 1'b0; end
    end endgenerate
endmodule
