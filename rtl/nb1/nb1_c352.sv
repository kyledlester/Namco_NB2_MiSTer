// Namco NB-1 MiSTer core -- Namco C352 PCM sound chip (M15).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Behavioural contract = MAME 0.289 src/devices/sound/c352.cpp (docs/M15_RESEARCH.md
// section 1); scripts/research/nb1_m15_c352.py restates it and equals MAME's own
// 84 kHz output sample for sample on a 60 s Nebulas Ray capture. The sequencer
// structure (tick, voices in order, one cached 8-byte ROM line per voice, blocking
// fetch) follows the NA-1/NA-2 core's na1_c219; the arithmetic is the C352's.
//
//   32 voices; registers (word offsets) voice v: v*8 + {0 vol_f (L<<8|R), 1 vol_r
//   (rear, stored/readable only), 2 freq, 3 flags, 4 wave_bank, 5 wave_start, 6 wave_end,
//   7 wave_loop}; 0x200 control (stored); 0x202 key-on/off execute (16-bit writes only).
//   Per output sample, voices 0..31: if BUSY: next = counter + freq; bit 16 -> fetch
//   (8-bit linear << 8, mu-law, or the shared noise LFSR; then the position rules:
//   ping-pong, link, loop, key-off at the end); (next ^ counter) & 0x18000 -> ramp the
//   front current volumes one step toward their targets; counter = next[15:0];
//   interpolate last + (counter * (sample - last) >> 16) unless FILTER; mix
//   (+/-s * vol) >> 8 per side (PHASEFL left, PHASEFR right). Output s16(sum >> 3)
//   (16-bit wrap, as MAME). Outputs 2/3 (rear) are not connected on the NB-1, so the
//   rear current volumes are not kept (never readable, never heard).
//
// Access timing [IMPLEMENTATION, = MAME's stream rule]: C75 writes are posted in a
// 16-entry FIFO and applied only while no sample is being rendered, so every write
// affects the samples rendered after it. Reads wait only for the posted writes (read
// after write): flags, control and unmapped offsets (flip-flops) are answered one clock
// after the FIFO is empty, even during a render (M16: the firmware polls flags ~900
// times/s, and waiting for the render stalled the C75 beyond MAME's cycle budget,
// docs/M16_IMPLEMENTATION.md); a read of a RAM-held register (never done by Nebulas Ray)
// also waits for an idle engine, which owns the RAM read port during a render. Key-on/off at 0x202 updates all flags at
// once; the key-on's play-state reset is applied at the voice's next slot (it is not
// observable earlier). Voice ROM: SDRAM region VOICE (2 MiB populated; MAME's 16 MiB
// window reads 0 above it) through one 8-byte line per voice.
module nb1_c352 (
    input  wire               clk_sys,
    input  wire               reset,
    input  wire               sample_tick,      // 84 kHz (clk_sys / 1152)

    // C75 port: held request, one-cycle ack (nb1_c75 bus contract)
    input  wire               bus_req,
    input  wire               bus_we,
    input  wire [10:0]        bus_addr,         // word offset (C75 address - $2000) / 2
    input  wire [1:0]         bus_be,           // [1] D15:8, [0] D7:0
    input  wire [15:0]        bus_wdata,
    output reg                bus_ack = 1'b0,
    output reg  [15:0]        bus_rdata = '0,

    // voice ROM reads (nb1_memory client contract, region VOICE)
    output reg                mreq_valid = 1'b0,
    input  wire               mreq_ready,
    output wire [3:0]         mreq_region,      // always REG_VOICE
    output reg  [24:0]        mreq_offset = '0,
    input  wire               mrsp_valid,
    input  wire [63:0]        mrsp_line,
    input  wire               mrsp_err,

    output reg  signed [15:0] out_l = '0,
    output reg  signed [15:0] out_r = '0,
    output reg                out_valid = 1'b0,

    // diagnostics
    output reg  [15:0]        overruns = '0,    // ticks that found the previous one still pending
    output reg  [31:0]        fetches = '0,     // voice-ROM line reads
    output reg  [5:0]         busy_voices = '0, // BUSY voices after the last rendered sample
    output reg  [15:0]        max_wait = '0     // longest voice-ROM read, clk_sys
);
    import nb1_mem_pkg::*;
    assign mreq_region = REG_VOICE;

    localparam [15:0] F_BUSY = 16'h8000, F_KEYON = 16'h4000, F_KEYOFF = 16'h2000, F_LOOPHIST = 16'h0800;
    localparam int B_PHASEFL = 8, B_PHASEFR = 7, B_LDIR = 6, B_LINK = 5, B_NOISE = 4, B_MULAW = 3,
                   B_FILTER = 2, B_LOOP = 1, B_REVERSE = 0;

    // ---------------------------------------------------------------- register file
    // Words 0,1,2,4-7 of every voice (index {v, reg}) in one simple-dual-port M10K:
    // written by the FIFO drain, read by the engine or, while idle, by a C75 read.
    (* ramstyle = "M10K, no_rw_check" *) reg [1:0][7:0] regs [0:255];
    reg  [7:0]  r_waddr = '0;
    reg  [15:0] r_wdata = '0;
    reg  [1:0]  r_wbe = '0;
    reg         r_we = 1'b0;
    reg  [7:0]  r_raddr = '0;
    reg  [15:0] r_q = '0;
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (int i = 0; i < 256; i++) regs[i] = '0;
`endif
// synthesis translate_on
    always @(posedge clk_sys) begin
        if (r_we) begin
            if (r_wbe[1]) regs[r_waddr][1] <= r_wdata[15:8];
            if (r_wbe[0]) regs[r_waddr][0] <= r_wdata[7:0];
        end
        r_q <= regs[r_raddr];
    end

    // ---------------------------------------------------------------- per-voice state (flip-flops)
    reg [15:0] flags   [0:31];
    // M16: the line cache, its tags and the write FIFO are small; keep them out of M10K
    // (548/553 blocks in M15) -- MLAB (LUT RAM) or registers.
    reg [23:0] v_pos   [0:31];
    reg [15:0] v_cnt   [0:31];
    reg signed [15:0] v_smp [0:31], v_last [0:31];
    reg [7:0]  v_cv0   [0:31], v_cv1 [0:31];
    reg        v_init  [0:31];
    reg        c_valid [0:31];
    (* ramstyle = "MLAB" *) reg [20:0] c_tag  [0:31];
    (* ramstyle = "MLAB" *) reg [63:0] c_data [0:31];
    reg [15:0] control = '0;
    reg [15:0] lfsr = 16'h1234;
    integer i;
// synthesis translate_off
`ifndef SYNTHESIS
    initial for (i = 0; i < 32; i++) begin
        flags[i] = '0; v_pos[i] = '0; v_cnt[i] = '0; v_smp[i] = '0; v_last[i] = '0; v_cv0[i] = '0; v_cv1[i] = '0;
        v_init[i] = 1'b0; c_valid[i] = 1'b0; c_tag[i] = '0; c_data[i] = '0;
    end
`endif
// synthesis translate_on

    // ---------------------------------------------------------------- write FIFO (posted C75 writes)
    (* ramstyle = "MLAB" *) reg [28:0] fifo [0:15];      // {addr[10:0], data[15:0], be[1:0]}
    reg [3:0]  f_rd = '0, f_wr = '0;
    reg [4:0]  f_cnt = '0;
    wire       f_empty = (f_cnt == 5'd0);
    wire       f_full  = (f_cnt == 5'd16);
    wire [28:0] f_head = fifo[f_rd];
    // reads answered from flip-flops (no register RAM): flags, control, unmapped
    wire        rd_fast = (bus_addr >= 11'h100) || (bus_addr[2:0] == 3'd3);

    // ---------------------------------------------------------------- mu-law (MAME m_mulawtab)
    function automatic signed [15:0] mulaw(input [7:0] b);
        reg [6:0] k; reg [10:0] j;
        begin
            k = b[6:0];
            if (k < 7'd16)       j = {4'd0, k};
            else if (k < 7'd24)  j = 11'd16  + {3'd0, k - 7'd16, 1'b0};
            else if (k < 7'd48)  j = 11'd32  + {2'd0, k - 7'd24, 2'b0};
            else if (k < 7'd100) j = 11'd128 + {1'b0, k - 7'd48, 3'b0};
            else                 j = 11'd544 + {k - 7'd100, 4'b0};
            // positive half j << 5; negative half (~(j << 5)) & 0xFFE0 = -(j + 1) << 5
            mulaw = b[7] ? -$signed({j + 11'd1, 5'b0}) : $signed({j, 5'b0});
        end
    endfunction

    // ---------------------------------------------------------------- engine
    localparam [3:0] S_IDLE = 4'd0, S_RD1 = 4'd1, S_RD2 = 4'd2, S_V0 = 4'd3, S_REGS = 4'd4, S_CALC = 4'd5,
                     S_FETCH = 4'd6, S_MEM = 4'd7, S_DEC = 4'd8, S_RAMP = 4'd9, S_INT = 4'd10, S_INT2 = 4'd11,
                     S_MIX = 4'd12, S_ACC = 4'd13, S_OUT = 4'd14;
    reg [3:0]  st = S_IDLE;
    reg        tick_pending = 1'b0;
    reg [4:0]  v = '0;
    reg [3:0]  k = '0;                 // register read index
    reg [10:0] rd_addr = '0;
    // working copy of the current voice
    reg [15:0] w_flags, w_volf, w_freq, w_bank, w_start, w_end, w_loop;
    reg [23:0] w_pos;
    reg [15:0] w_cnt;
    reg signed [15:0] w_smp, w_last;
    reg [7:0]  w_cv0, w_cv1;
    reg [16:0] w_next;
    reg        w_ramp;
    reg [7:0]  w_byte;
    reg signed [32:0] w_prod;
    reg signed [15:0] w_s;
    reg signed [25:0] w_m0, w_m1;
    reg signed [23:0] acc0, acc1;
    reg [5:0]  nbusy;
    reg [15:0] wait_cnt;
    // Accepted read not yet answered. NOT cleared by reset: after a reset during a fetch the old
    // response must go by before a new request (nb1_memory: one outstanding read per client).
    reg        mem_pend = 1'b0;
    always @(posedge clk_sys) begin
        if (mreq_valid && mreq_ready) mem_pend <= 1'b1;
        else if (mrsp_valid)          mem_pend <= 1'b0;
    end

    wire [15:0] lfsr_next = (lfsr >> 1) ^ ({16{lfsr[0]}} & 16'hFFF6);
    wire [20:0] line_of_pos = w_pos[23:3];
    wire        rom_oob = (w_pos[23:21] != 3'd0);          // >= 2 MiB: MAME's zero-filled window
    function automatic [7:0] line_byte(input [63:0] d, input [2:0] a);
        line_byte = d[63 - 8 * a -: 8];                     // byte A+0 in bits 63:56 (nb1_memory)
    endfunction
    function automatic [7:0] ramp(input [7:0] cur, input [7:0] tgt);
        ramp = (cur > tgt) ? cur - 8'd1 : (cur < tgt) ? cur + 8'd1 : cur;
    endfunction
    wire signed [16:0] w_diff = $signed({w_smp[15], w_smp}) - $signed({w_last[15], w_last});
    wire signed [16:0] w_sneg0 = w_flags[B_PHASEFL] ? -$signed({w_s[15], w_s}) : $signed({w_s[15], w_s});
    wire signed [16:0] w_sneg1 = w_flags[B_PHASEFR] ? -$signed({w_s[15], w_s}) : $signed({w_s[15], w_s});

    always @(posedge clk_sys) begin
        bus_ack   <= 1'b0;
        out_valid <= 1'b0;
        r_we      <= 1'b0;
        if (reset) begin
            st <= S_IDLE; tick_pending <= 1'b0; f_rd <= '0; f_wr <= '0; f_cnt <= '0;
            control <= '0; lfsr <= 16'h1234; mreq_valid <= 1'b0; overruns <= '0; fetches <= '0;
            busy_voices <= '0; max_wait <= '0; out_l <= '0; out_r <= '0;
            for (i = 0; i < 32; i++) begin
                flags[i] <= '0; v_init[i] <= 1'b0; c_valid[i] <= 1'b0; v_cv0[i] <= '0; v_cv1[i] <= '0;
                v_pos[i] <= '0; v_cnt[i] <= '0; v_smp[i] <= '0; v_last[i] <= '0;
            end
        end else begin
            // -------- sample cadence
            if (sample_tick) begin
                if (tick_pending) overruns <= overruns + 16'd1;
                tick_pending <= 1'b1;
            end
            // -------- C75 fast reads: after the posted writes, in any engine state
            if (bus_req && !bus_we && !bus_ack && rd_fast && f_empty && st != S_RD1 && st != S_RD2) begin
                bus_rdata <= (bus_addr < 11'h100) ? flags[bus_addr[7:3]] :
                             (bus_addr == 11'h200) ? control : 16'h0000;
                bus_ack <= 1'b1;
            end
            // -------- C75 writes: posted (acknowledged when queued)
            if (bus_req && bus_we && !bus_ack && !f_full) begin
                fifo[f_wr] <= {bus_addr, bus_wdata, bus_be};
                f_wr <= f_wr + 4'd1;
                bus_ack <= 1'b1;
            end
            f_cnt <= f_cnt + ((bus_req && bus_we && !bus_ack && !f_full) ? 5'd1 : 5'd0)
                           - ((st == S_IDLE && !f_empty) ? 5'd1 : 5'd0);

            case (st)
            S_IDLE: begin
                if (!f_empty) begin
                    // apply the oldest posted write (never during a sample)
                    f_rd <= f_rd + 4'd1;
                    if (f_head[28:18] < 11'h100) begin
                        if (f_head[20:18] == 3'd3) begin
                            if (f_head[1]) flags[f_head[25:21]][15:8] <= f_head[17:10];
                            if (f_head[0]) flags[f_head[25:21]][7:0]  <= f_head[9:2];
                        end else begin
                            r_we <= 1'b1; r_waddr <= f_head[25:18]; r_wdata <= f_head[17:2]; r_wbe <= f_head[1:0];
                        end
                    end else if (f_head[28:18] == 11'h200) begin
                        if (f_head[1]) control[15:8] <= f_head[17:10];
                        if (f_head[0]) control[7:0]  <= f_head[9:2];
                    end else if (f_head[28:18] == 11'h202 && f_head[1:0] == 2'b11) begin
                        // execute key-ons / key-offs (MAME c352_device::write offset 0x202)
                        for (i = 0; i < 32; i++) begin
                            if (flags[i][14]) begin
                                if (flags[i][13]) flags[i] <= flags[i] & ~(F_BUSY | F_KEYON | F_KEYOFF | F_LOOPHIST);
                                else              flags[i] <= (flags[i] | F_BUSY) & ~(F_KEYON | F_LOOPHIST);
                                v_init[i] <= 1'b1;
                            end else if (flags[i][13]) begin
                                flags[i] <= flags[i] & ~(F_BUSY | F_KEYOFF);
                            end
                        end
                    end
                end else if (bus_req && !bus_we && !bus_ack && !rd_fast) begin
                    rd_addr <= bus_addr; r_raddr <= bus_addr[7:0]; st <= S_RD1;
                end else if (tick_pending) begin
                    tick_pending <= 1'b0; v <= '0; acc0 <= '0; acc1 <= '0; nbusy <= '0; st <= S_V0;
                end
            end
            S_RD1: st <= S_RD2;                      // register RAM read in flight
            S_RD2: begin
                bus_rdata <= (rd_addr < 11'h100) ? ((rd_addr[2:0] == 3'd3) ? flags[rd_addr[7:3]] : r_q) :
                             (rd_addr == 11'h200) ? control : 16'h0000;
                bus_ack <= 1'b1;
                st <= S_IDLE;
            end
            // -------- one voice
            S_V0: begin
                w_flags <= flags[v];
                if (!flags[v][15]) begin
                    v_init[v] <= 1'b0;                   // keyed on and off in one execute
                    st <= (v == 5'd31) ? S_OUT : S_V0;
                    v <= v + 5'd1;
                end else begin
                    k <= 4'd0; r_raddr <= {v, 3'd0}; st <= S_REGS;
                end
            end
            S_REGS: begin
                // read words 0..7 of voice v (data one clock after the address)
                if (k < 4'd7) r_raddr <= {v, k[2:0] + 3'd1};
                case (k)
                    4'd1: w_volf  <= r_q;
                    4'd3: w_freq  <= r_q;
                    4'd5: w_bank  <= r_q;
                    4'd6: w_start <= r_q;
                    4'd7: w_end   <= r_q;
                    4'd8: w_loop  <= r_q;
                    default: ;
                endcase
                k <= k + 4'd1;
                if (k == 4'd8) st <= S_CALC;
            end
            S_CALC: begin
                if (v_init[v]) begin
                    // key-on play state (MAME: pos = bank:start, samples 0, counter 0xFFFF, volumes 0)
                    v_init[v] <= 1'b0;
                    w_pos <= {w_bank[7:0], w_start}; w_cnt <= 16'hFFFF; w_smp <= '0; w_last <= '0;
                    w_cv0 <= '0; w_cv1 <= '0;
                    w_next <= {1'b0, 16'hFFFF} + {1'b0, w_freq};
                    w_ramp <= |((({1'b0, 16'hFFFF} + {1'b0, w_freq}) ^ {1'b0, 16'hFFFF}) & 17'h18000);
                    st <= ((({1'b0, 16'hFFFF} + {1'b0, w_freq}) & 17'h10000) != 0) ? S_FETCH : S_RAMP;
                end else begin
                    w_pos <= v_pos[v]; w_cnt <= v_cnt[v]; w_smp <= v_smp[v]; w_last <= v_last[v];
                    w_cv0 <= v_cv0[v]; w_cv1 <= v_cv1[v];
                    w_next <= {1'b0, v_cnt[v]} + {1'b0, w_freq};
                    w_ramp <= |((({1'b0, v_cnt[v]} + {1'b0, w_freq}) ^ {1'b0, v_cnt[v]}) & 17'h18000);
                    st <= ((({1'b0, v_cnt[v]} + {1'b0, w_freq}) & 17'h10000) != 0) ? S_FETCH : S_RAMP;
                end
            end
            S_FETCH: begin
                if (w_flags[B_NOISE]) begin
                    lfsr <= lfsr_next; w_last <= w_smp; w_smp <= $signed(lfsr_next); st <= S_RAMP;
                end else if (rom_oob) begin
                    w_byte <= 8'd0; st <= S_DEC;
                end else if (c_valid[v] && c_tag[v] == line_of_pos) begin
                    w_byte <= line_byte(c_data[v], w_pos[2:0]); st <= S_DEC;
                end else if (!mem_pend) begin
                    mreq_valid <= 1'b1; mreq_offset <= {4'd0, w_pos[20:3], 3'b000}; wait_cnt <= '0; st <= S_MEM;
                end
            end
            S_MEM: begin
                wait_cnt <= wait_cnt + 16'd1;
                if (mreq_valid && mreq_ready) mreq_valid <= 1'b0;
                if (mrsp_valid && !mreq_valid) begin
                    fetches <= fetches + 32'd1;
                    if (wait_cnt > max_wait) max_wait <= wait_cnt;
                    c_valid[v] <= !mrsp_err; c_tag[v] <= line_of_pos; c_data[v] <= mrsp_line;
                    w_byte <= mrsp_err ? 8'd0 : line_byte(mrsp_line, w_pos[2:0]);
                    st <= S_DEC;
                end
            end
            S_DEC: begin
                // decode, then advance the position (MAME fetch_sample)
                w_last <= w_smp;
                w_smp  <= w_flags[B_MULAW] ? mulaw(w_byte) : $signed({w_byte, 8'h00});
                if (w_flags[B_LOOP] && w_flags[B_REVERSE]) begin
                    if (w_flags[B_LDIR] && w_pos[15:0] == w_loop) begin
                        w_flags[B_LDIR] <= 1'b0; w_pos <= w_pos + 24'd1;
                    end else if (!w_flags[B_LDIR] && w_pos[15:0] == w_end) begin
                        w_flags[B_LDIR] <= 1'b1; w_pos <= w_pos - 24'd1;
                    end else
                        w_pos <= w_flags[B_LDIR] ? w_pos - 24'd1 : w_pos + 24'd1;
                end else if (w_pos[15:0] == w_end) begin
                    if (w_flags[B_LINK] && w_flags[B_LOOP]) begin
                        w_pos <= {w_start[7:0], w_loop}; w_flags <= w_flags | F_LOOPHIST;
                    end else if (w_flags[B_LOOP]) begin
                        w_pos <= {w_pos[23:16], w_loop}; w_flags <= w_flags | F_LOOPHIST;
                    end else begin
                        w_flags <= (w_flags | F_KEYOFF) & ~F_BUSY;
                        w_smp <= '0;
                    end
                end else
                    w_pos <= w_flags[B_REVERSE] ? w_pos - 24'd1 : w_pos + 24'd1;
                st <= S_RAMP;
            end
            S_RAMP: begin
                if (w_ramp) begin
                    w_cv0 <= ramp(w_cv0, w_volf[15:8]);
                    w_cv1 <= ramp(w_cv1, w_volf[7:0]);
                end
                w_cnt <= w_next[15:0];
                st <= S_INT;
            end
            S_INT: begin
                w_prod <= $signed({1'b0, w_cnt}) * w_diff;       // counter * (sample - last)
                st <= S_INT2;
            end
            S_INT2: begin
                w_s <= w_flags[B_FILTER] ? w_smp : w_last + w_prod[31:16];   // >> 16 (floor), s16
                st <= S_MIX;
            end
            S_MIX: begin
                w_m0 <= w_sneg0 * $signed({1'b0, w_cv0});
                w_m1 <= w_sneg1 * $signed({1'b0, w_cv1});
                st <= S_ACC;
            end
            S_ACC: begin
                acc0 <= acc0 + 24'(w_m0 >>> 8);
                acc1 <= acc1 + 24'(w_m1 >>> 8);
                if (w_flags[15]) nbusy <= nbusy + 6'd1;
                flags[v] <= w_flags; v_pos[v] <= w_pos; v_cnt[v] <= w_cnt; v_smp[v] <= w_smp; v_last[v] <= w_last;
                v_cv0[v] <= w_cv0; v_cv1[v] <= w_cv1;
                st <= (v == 5'd31) ? S_OUT : S_V0;
                v <= v + 5'd1;
            end
            S_OUT: begin
                out_l <= acc0[18:3]; out_r <= acc1[18:3];         // s16(sum >> 3): MAME's wrap
                out_valid <= 1'b1;
                busy_voices <= nbusy;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
