// Namco NB-1 MiSTer core -- two read-only requesters on one nb1_memory client port (M15).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The C352 voice-ROM reads (B, ~1 per video line on average, latency-sensitive) share the C75 data-ROM
// client port (A) instead of adding an arbiter client (docs/M15_PLAN.md): the arbiter, its priorities and
// the renderers are unchanged. nb1_memory contract per requester: valid held with region/offset until
// ready; one read outstanding; the response (rsp_valid with the shared line/err buses) returns later.
// One request is outstanding at the port at a time; the owner of that request receives rsp_valid.
// When both wait, B (audio) goes first: its one-sample deadline is the tighter one.
module nb1_mem_mux2 (
    input  wire        clk_sys,

    input  wire        a_valid,
    output wire        a_ready,
    input  wire [3:0]  a_region,
    input  wire [24:0] a_offset,
    output wire        a_rsp_valid,

    input  wire        b_valid,
    output wire        b_ready,
    input  wire [3:0]  b_region,
    input  wire [24:0] b_offset,
    output wire        b_rsp_valid,

    output wire        m_valid,
    input  wire        m_ready,
    output wire [3:0]  m_region,
    output wire [24:0] m_offset,
    input  wire        m_rsp_valid
);
    // owner of the request presented / outstanding: 0 = A, 1 = B
    reg busy = 1'b0;        // a request was accepted and its response has not come back
    reg own  = 1'b0;        // owner of the accepted (or presented, while holding) request
    reg hold = 1'b0;        // a request is being presented (sticky until accepted)
    wire pick_b = hold ? own : b_valid;
    wire any    = hold ? 1'b1 : (a_valid || b_valid);

    assign m_valid  = !busy && any && (pick_b ? b_valid : a_valid);
    assign m_region = pick_b ? b_region : a_region;
    assign m_offset = pick_b ? b_offset : a_offset;
    assign a_ready  = m_valid && m_ready && !pick_b;
    assign b_ready  = m_valid && m_ready &&  pick_b;
    assign a_rsp_valid = m_rsp_valid && busy && !own;
    assign b_rsp_valid = m_rsp_valid && busy &&  own;

    always @(posedge clk_sys) begin
        if (!busy && !hold && (a_valid || b_valid)) begin hold <= 1'b1; own <= b_valid; end
        if (m_valid && m_ready) begin busy <= 1'b1; hold <= 1'b0; own <= pick_b; end
        else if (m_rsp_valid && busy) busy <= 1'b0;
        // a presented request withdrawn by its (reset) requester: release the hold
        if (hold && !busy && !(own ? b_valid : a_valid)) hold <= 1'b0;
    end
endmodule
