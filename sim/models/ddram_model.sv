// Namco NB-2 MiSTer core -- behavioural model of the MiSTer DDRAM port (f2sdram Avalon-MM, 64-bit words).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Accepts one command per clock while DDRAM_BUSY is low (random BUSY cycles), burst count 1 only; read data
// returns in command order after a random latency (LAT_MIN..LAT_MAX clocks, with occasional long stalls of
// STALL clocks to imitate HPS traffic). Storage covers WORDS 64-bit words from BASE_WORD; reads elsewhere
// return X and are counted as errors. Unwritten words read UNWRITTEN.
module ddram_model #(
    parameter int  BASE_WORD = 29'h6400000,   // byte 0x32000000
    parameter int  WORDS     = 20 * 1024 * 1024 / 8,
    parameter int  LAT_MIN   = 12,
    parameter int  LAT_MAX   = 30,
    parameter int  STALL     = 300,
    parameter int  STALL_PPM = 200,           // chance per read of a long stall (parts per million)
    parameter int  BUSY_PCT  = 20,
    parameter [63:0] UNWRITTEN = 64'h5A5A_C3C3_A5A5_3C3C
) (
    input  wire        clk,
    output reg         busy = 1'b0,
    input  wire [7:0]  burstcnt,
    input  wire [28:0] addr,
    output reg  [63:0] dout = '0,
    output reg         dout_ready = 1'b0,
    input  wire        rd,
    input  wire [63:0] din,
    input  wire [7:0]  be,
    input  wire        we
);
    bit [63:0] mem [0:WORDS-1];
    initial for (int i = 0; i < WORDS; i++) mem[i] = UNWRITTEN;
    integer errors = 0, reads = 0, writes = 0;
    int unsigned seed = 7;
    function automatic int unsigned rnd();
        seed = seed * 1103515245 + 12345;
        return seed >> 8;
    endfunction
    longint cyc = 0;
    // pending reads: data and the clock at which it may return
    typedef struct { bit [63:0] d; longint t; } rd_t;
    rd_t q [$];
    longint last_t = 0;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        dout_ready <= 1'b0;
        if ((rd || we) && !busy) begin
            automatic longint w = longint'(addr) - BASE_WORD;
            if (burstcnt != 8'd1) begin errors++; $display("DDRAM-MODEL ERROR: burst count %0d", burstcnt); end
            if (rd && we) begin errors++; $display("DDRAM-MODEL ERROR: RD and WE together"); end
            if (w < 0 || w >= WORDS) begin
                errors++;
                if (errors < 10) $display("DDRAM-MODEL ERROR: access outside the store, word %h", addr);
            end else if (we) begin
                writes++;
                for (int k = 0; k < 8; k++) if (be[k]) mem[w][8*k +: 8] = din[8*k +: 8];
            end else begin
                rd_t e;
                longint t;
                reads++;
                t = cyc + LAT_MIN + rnd() % (LAT_MAX - LAT_MIN + 1);
                if (rnd() % 1000000 < STALL_PPM) t += STALL;
                if (t <= last_t) t = last_t + 1;          // in order, one word per clock
                last_t = t;
                e.d = mem[w]; e.t = t;
                q.push_back(e);
            end
        end
        if (q.size() != 0 && q[0].t <= cyc) begin
            dout <= q[0].d;
            dout_ready <= 1'b1;
            void'(q.pop_front());
        end
        busy <= (rnd() % 100) < BUSY_PCT;
    end
endmodule
