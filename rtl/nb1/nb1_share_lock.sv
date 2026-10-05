// Namco NB-1 MiSTer core -- shared-RAM read-modify-write lock (M24).
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// The C75 sound driver takes a command from a shared-RAM mailbox word with a read-modify-write: it reads
// the word and, one or two instructions (~1-1.4 us) later, writes it back marked as taken (e.g. $40A1 ->
// $80A1). On this core both CPUs run truly in parallel, so a 68020 write of the NEXT command into the
// same word inside that window was overwritten by the C75's write-back and lost; a lost "release"
// command left a note sounding until the driver was reset (the M24 stuck organ note). MAME interleaves
// the CPUs in time slices, so the C75's read and write-back never have a 68020 write between them.
//
// This block gives the same atomicity: a C75 read of word W opens a window on W; a 68020 write to W
// arriving while it is open is held (m_wait) and performed (m_go) right after the C75 writes W, or when
// the window expires (WINDOW clk_sys after the read). While a 68020 write is held, C75 reads do not
// re-open the window, so a held write waits at most WINDOW clocks. Writes to other words, reads, and
// writes outside a window are not delayed.
module nb1_share_lock #(parameter int AW = 14, parameter int WINDOW = 511) (
    input  wire          clk_sys,
    input  wire          reset,
    // C75 port of the shared RAM
    input  wire          c_en,
    input  wire          c_we,
    input  wire [AW-1:0] c_addr,
    // 68020: a shared-RAM write starting this cycle, and its word address (held stable while waiting)
    input  wire          m_wr,
    input  wire [AW-1:0] m_addr,
    output wire          m_wait,      // hold this write (do not write the RAM, do not complete the cycle)
    output wire          m_go,        // perform the held write now and complete the cycle
    output reg  [15:0]   holds = '0   // diagnostics: writes held (saturating)
);
    localparam int CW = $clog2(WINDOW + 1);
    reg [AW-1:0] w_addr = '0;
    reg [CW-1:0] age = CW'(WINDOW);          // WINDOW = closed
    reg          pend = 1'b0;
    reg          c_wback_q = 1'b0;    // the C75 wrote the window's word back last cycle
    wire         open    = (age != CW'(WINDOW));
    wire         c_wback = c_en && c_we && (c_addr == w_addr);
    // NB-2: no C75 term in m_wait (timing: the C75 address decode reached the 68020 clock enable through it).
    // A 68020 write that meets the C75's write-back in the same clock is now held one clock and performed
    // right after it (the write-back closes the window, so m_go follows), instead of both writing that clock.
    assign m_wait = m_wr && open && (m_addr == w_addr);
    // the C75 wrote the word back (its write lands this cycle; ours goes next cycle), or the window ran out
    assign m_go   = pend && (!open || c_wback_q);
    always @(posedge clk_sys) begin
        c_wback_q <= c_wback;
        if (reset) begin
            age <= CW'(WINDOW); pend <= 1'b0; c_wback_q <= 1'b0;
        end else begin
            if (open) age <= age + 1'b1;
            if (c_wback) age <= CW'(WINDOW);
            else if (c_en && !c_we && !pend && !m_wait) begin w_addr <= c_addr; age <= '0; end
            if (m_wait) begin pend <= 1'b1; if (holds != 16'hFFFF) holds <= holds + 16'd1; end
            if (m_go)   pend <= 1'b0;
        end
    end
endmodule
