// Namco NB-1 MiSTer core -- 68020 program-ROM cache (M12).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Direct-mapped cache of 2**LINES_LOG2 lines x 8 bytes over the read-only
// PROG region ($000000-$0FFFFF), for the 68020's instruction fetches and ROM
// data reads only (nb1_main_bus decides what is a ROM read). Size study and
// choice: docs/M12_RESEARCH.md (512 lines recommended; 256/1,024 are rebuilds).
//
// Storage: one simple dual-port array of 80-bit words {tag = addr[19:4],
// data = the aligned 8-byte line, big-endian as nb1_memory rsp_line} (512
// lines = 4 M10K in 512 x 20 mode). The tag keeps every address bit above the
// line offset (more than the index needs at 512 lines; it keeps the entry
// self-describing and the width at exactly 80 bits).
//
// Use (nb1_main_bus): an L1 behind the unchanged M3-M11 2-line zero-latency
// buffer (L0). Only L0 misses look up here; the whole 8-byte line is returned
// so L0 can be refilled. (A cache with a 2-clk hit on every fetch was measured
// to cost the TG68K 20 % of its nominal rate in a tight loop: M3 sched test.)
//
// Bypass (runtime): no lookup and no fill; every request goes straight to
// SDRAM, i.e. the M3-M11 behaviour (plus one clock per miss). Changing
// `bypass` or `reset` invalidates all lines.
//
// Timing (clk_sys):
//   req  (one-cycle pulse; addr held stable until done)  -> array read
//   +1   tag compare                                      -> hit: rdata/done registered
//   +2   done pulse (hit)
//   miss: one 8-byte nb1_memory read (client contract, one outstanding); the
//   line is written into the array and the word returned from rsp_line, done
//   the clock after the response. rsp_err: word $FFFF, err = 1, no fill.
// A response still outstanding across a reset is discarded (never filled).
// PROG is read-only while the CPU runs, so there is no write invalidation.
module nb1_prog_cache #(parameter int LINES_LOG2 = 9) (
    input  wire        clk_sys,
    input  wire        reset,           // invalidate all lines, abandon a lookup
    input  wire        bypass,          // 1 = 2-line geometry (M3-M11 behaviour)

    input  wire        req,             // lookup request (one clock)
    input  wire [19:1] addr,            // word address inside PROG
    output reg         done = 1'b0,     // one-clock pulse: rdata/err valid from now until the next req
    output reg  [15:0] rdata = 16'hFFFF,
    output reg  [63:0] line = '0,        // the whole aligned 8-byte line (for the L0 refill)
    output reg         err = 1'b0,
    output reg         hit_pulse = 1'b0,
    output reg         miss_pulse = 1'b0,

    // nb1_memory client port (region PROG supplied by the parent)
    output reg         mreq_valid = 1'b0,
    input  wire        mreq_ready,
    output reg  [24:0] mreq_offset = '0,
    input  wire        mrsp_valid,
    input  wire [63:0] mrsp_line,
    input  wire        mrsp_err
);
    localparam int LINES = 1 << LINES_LOG2;
    localparam int IW    = LINES_LOG2;

    // ------------------------------------------------------------ storage
    (* ramstyle = "M10K, no_rw_check" *) reg [79:0] mem [0:LINES-1];
    reg  [79:0]      q = '0;
    reg              we = 1'b0;
    reg  [IW-1:0]    waddr = '0;
    reg  [79:0]      wdata = '0;
    reg  [LINES-1:0] valid = '0;

    wire [IW-1:0] idx = addr[IW+2:3];

    always @(posedge clk_sys) begin
        if (we) mem[waddr] <= wdata;
        if (req) q <= mem[idx];
    end

    // ------------------------------------------------------------ control
    localparam [1:0] S_IDLE = 2'd0, S_LOOK = 2'd1, S_FILL = 2'd2;
    reg [1:0]    st = S_IDLE;
    reg [IW-1:0] idx_r = '0;
    reg [15:0]   tag_r = '0;     // addr[19:4] (all bits above the line offset)
    reg [1:0]    sel_r = '0;     // addr[2:1]
    reg          pend = 1'b0;    // an nb1_memory request is accepted and unanswered
    reg          mine = 1'b0;    // ... and it belongs to the current fill
    reg          bypass_q = 1'b0;

    function automatic [15:0] word_of(input [63:0] line, input [1:0] sel);
        case (sel)
            2'd0: word_of = line[63:48];
            2'd1: word_of = line[47:32];
            2'd2: word_of = line[31:16];
            default: word_of = line[15:0];
        endcase
    endfunction

    wire look_hit = valid[idx_r] && (q[79:64] == tag_r);

    always @(posedge clk_sys) begin
        we         <= 1'b0;
        done       <= 1'b0;
        hit_pulse  <= 1'b0;
        miss_pulse <= 1'b0;
        bypass_q   <= bypass;

        // nb1_memory handshake (also across a reset)
        if (mreq_valid && mreq_ready) begin
            mreq_valid <= 1'b0;
            pend       <= 1'b1;
        end
        if (pend && mrsp_valid) pend <= 1'b0;

        case (st)
            S_IDLE: ;
            S_LOOK: begin
                if (look_hit) begin
                    rdata     <= word_of(q[63:0], sel_r);
                    line      <= q[63:0];
                    err       <= 1'b0;
                    done      <= 1'b1;
                    hit_pulse <= 1'b1;
                    st        <= S_IDLE;
                end else begin
                    miss_pulse <= 1'b1;
                    st         <= S_FILL;
                end
            end
            S_FILL: begin
                // issue once no older response is outstanding
                if (!mine && !pend && !mreq_valid) begin
                    mreq_valid  <= 1'b1;
                    mreq_offset <= {5'd0, tag_r, idx_r[0], 3'b000};   // line = addr[19:3]; idx_r[0] = addr[3] in both geometries
                    mine        <= 1'b1;
                end
                if (mine && pend && mrsp_valid) begin
                    mine <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                    if (mrsp_err) begin
                        rdata <= 16'hFFFF;
                        line  <= '0;
                        err   <= 1'b1;
                    end else begin
                        rdata <= word_of(mrsp_line, sel_r);
                        line  <= mrsp_line;
                        err   <= 1'b0;
                        we    <= !bypass;
                        waddr <= idx_r;
                        wdata <= {tag_r, mrsp_line};
                        valid[idx_r] <= !bypass;
                    end
                end
            end
            default: st <= S_IDLE;
        endcase

        if (req) begin
            idx_r <= idx;
            tag_r <= addr[19:4];
            sel_r <= addr[2:1];
            st    <= bypass ? S_FILL : S_LOOK;      // bypass: straight to SDRAM
            if (bypass) miss_pulse <= 1'b1;
        end

        // invalidate: reset, or a change of geometry
        if (reset || (bypass != bypass_q)) begin
            valid <= '0;
            we    <= 1'b0;
        end
        if (reset) begin
            st   <= S_IDLE;
            mine <= 1'b0;
            if (!(mreq_valid && mreq_ready)) mreq_valid <= 1'b0;
        end
    end
endmodule
