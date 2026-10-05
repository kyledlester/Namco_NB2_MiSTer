// Namco NB-1 MiSTer core -- main 68EC020 interrupt controller, VBL (M9) and POS (M13) sources.
// Copyright (C) 2026 Kyle Lester. SPDX-License-Identifier: GPL-3.0-or-later
//
// Behavioural model of the VBL part of the NB-1 cpureg interrupt logic,
// exactly as MAME models it (namconb1_state::scantimer / cpureg_w /
// machine_reset, docs/M9_RESEARCH.md sections 2-3):
//
//   $400004 (byte)  write: the old VBL request is withdrawn, then the VBL
//                   level becomes data & $0F. Nebulas Ray writes $32 (level 2)
//                   and $00 (off). Bits 7-4 are ignored like MAME.
//   $400009 (byte)  write (any data): the VBL request is withdrawn (ack).
//   line 224        at the start of scanline 224 (vbl_event), if the level is
//                   nonzero, the VBL request is raised and HELD: MAME uses
//                   ASSERT_LINE, not HOLD_LINE, so the CPU's interrupt
//                   acknowledge does not withdraw it. Only the ack, a level
//                   write or a reset does. With level 0 an event is ignored
//                   (nothing is remembered for a later level write).
//   reset           level 0, nothing requested (MAME machine_reset).
//
// Reads of the cpureg bytes are not decoded here: MAME returns $FF for every
// cpureg read, which is what the bus's unimplemented path already returns.
//
// [IMPLEMENTATION-DECISION] Two cases MAME cannot tell us about:
//   * a level write or ack in the same clk_sys cycle as vbl_event: the event
//     wins (evaluated with the newly written level), so a VBL is never lost
//     to a coincident register write. Nebulas Ray never does this (its ack is
//     inside the handler, ~6 us after line 224).
//   * levels 8-15: MAME would pass them to set_input_line as nonexistent
//     68000 lines; the 68EC020 has three IPL pins. They are stored (and shown)
//     but request no interrupt. No NB-1 game in MAME writes them.
//
// M13: POS (raster position) source, same MAME model (namconb1_state::
// scantimer / cpureg_w, docs/M13_RESEARCH.md section 2):
//
//   $400001 (byte)  write: the old POS request is withdrawn, then the POS
//                   level becomes data & $0F (Nebulas Ray: $34 = level 4, $00).
//   $400006 (byte)  write (any data): the POS request is withdrawn (ack).
//   line S          at the start of every CPU scanline S (pos_event with
//                   pos_line = S), if S == C116 reg5 - 32 (unsigned; reg5 < 32
//                   never matches) and the level is nonzero, the POS request
//                   is raised and HELD until its ack, a level write or reset.
//   The raster here is the CPU raster (nb1_cpu_raster, video + 2 lines).
//   Levels: the 68020 sees the highest requested level of the two sources,
//   as MAME's separate input lines do.
// [IMPLEMENTATION-DECISION] as for VBL: an event coincident with a level write
// or ack wins; levels 8-15 are stored but request nothing. MAME clears the
// shared CPU input line of the old level; if the game ever gave POS and VBL
// the same level, a level write would there also withdraw a pending VBL. The
// two requests are kept separate here (Nebulas Ray uses 4 and 2).
//
// The unknown source ($400002/$400007) is not modelled (never enabled).
module nb1_main_irq (
    input  wire        clk_sys,
    input  wire        reset,          // CPU reset
    input  wire        vbl_event,      // one clk_sys pulse at the start of line 224
    input  wire        wr_level,       // byte write to $400004
    input  wire        wr_ack,         // byte write to $400009
    input  wire [7:0]  wdata,          // the byte written
    input  wire        iack,           // one pulse per CPU interrupt-acknowledge cycle
    input  wire [2:0]  iack_level,     // level being acknowledged
    output wire [2:0]  ipl,            // requested level (0 = none), to nb1_cpu

    // M13 POS source
    input  wire        pos_event,      // one clk_sys pulse at the start of every CPU line
    input  wire [8:0]  pos_line,       // the CPU line starting with pos_event
    input  wire [15:0] pos_reg5,       // C116 register 5 (POS line + 32)
    input  wire        wr_pos_level,   // byte write to $400001
    input  wire        wr_pos_ack,     // byte write to $400006
    input  wire [7:0]  pos_wdata,
    output reg  [3:0]  pos_level = 4'd0,
    output reg         pos_pending = 1'b0,
    output reg  [31:0] pos_raised = '0,   // POS requests raised since CPU reset
    output reg  [31:0] pos_taken = '0,    // CPU interrupt acknowledges at the POS level
    output reg  [31:0] pos_acks = '0,     // $400006 writes
    output reg  [8:0]  pos_frame = '0,    // POS requests raised in the last CPU frame (vbl_event to vbl_event)
    output reg  [8:0]  pos_ack_frame = '0,// $400006 writes in the last CPU frame
    output reg  [15:0] pos_odd_frames = '0, // CPU frames whose POS count is neither 0 nor one per
                                          // visible line (224), or whose ack count differs (saturates)

    // state and diagnostics (reset by CPU reset)
    output reg  [3:0]  vbl_level = 4'd0,
    output reg         vbl_pending = 1'b0,
    output reg  [31:0] vbl_events = '0,   // raster VBL events since CPU reset
    output reg  [31:0] vbl_raised = '0,   // events that raised the request (level != 0)
    output reg  [31:0] vbl_taken = '0,    // CPU interrupt acknowledges at the VBL level
    output reg  [31:0] vbl_acks = '0,     // $400009 writes
    output reg  [15:0] level_writes = '0  // $400004 writes
);
    wire [3:0] next_level = wr_level ? wdata[3:0] : vbl_level;
    wire       raise      = vbl_event && (next_level != 4'd0);

    wire [3:0]  pos_next  = wr_pos_level ? pos_wdata[3:0] : pos_level;
    wire [15:0] pos_cmp   = pos_reg5 - 16'd32;
    wire        pos_hit   = pos_event && (pos_reg5 >= 16'd32) && (pos_cmp == {7'd0, pos_line});
    wire        pos_raise = pos_hit && (pos_next != 4'd0);

    wire [2:0] vbl_ipl = (vbl_pending && vbl_level[3] == 1'b0) ? vbl_level[2:0] : 3'd0;
    wire [2:0] pos_ipl = (pos_pending && pos_level[3] == 1'b0) ? pos_level[2:0] : 3'd0;
    assign ipl = (pos_ipl > vbl_ipl) ? pos_ipl : vbl_ipl;

    reg [8:0] pos_frame_cnt = '0, pos_ack_cnt = '0;

    always @(posedge clk_sys) begin
        if (reset) begin
            vbl_level    <= 4'd0;
            vbl_pending  <= 1'b0;
            vbl_events   <= '0;
            vbl_raised   <= '0;
            vbl_taken    <= '0;
            vbl_acks     <= '0;
            level_writes <= '0;
        end else begin
            vbl_level <= next_level;
            if (raise)                  vbl_pending <= 1'b1;
            else if (wr_level || wr_ack) vbl_pending <= 1'b0;

            if (vbl_event) vbl_events   <= vbl_events + 32'd1;
            if (raise)     vbl_raised   <= vbl_raised + 32'd1;
            if (wr_ack)    vbl_acks     <= vbl_acks + 32'd1;
            if (wr_level)  level_writes <= level_writes + 16'd1;
            if (iack && vbl_level != 4'd0 && iack_level == vbl_level[2:0] && !vbl_level[3])
                vbl_taken <= vbl_taken + 32'd1;
        end
    end

    always @(posedge clk_sys) begin
        if (reset) begin
            pos_level     <= 4'd0;
            pos_pending   <= 1'b0;
            pos_raised    <= '0;
            pos_taken     <= '0;
            pos_acks      <= '0;
            pos_frame     <= '0;
            pos_ack_frame <= '0;
            pos_frame_cnt <= '0;
            pos_ack_cnt   <= '0;
            pos_odd_frames <= '0;
        end else begin
            pos_level <= pos_next;
            if (pos_raise)                        pos_pending <= 1'b1;
            else if (wr_pos_level || wr_pos_ack)  pos_pending <= 1'b0;

            if (pos_raise)  pos_raised <= pos_raised + 32'd1;
            if (wr_pos_ack) pos_acks   <= pos_acks + 32'd1;
            if (iack && pos_level != 4'd0 && iack_level == pos_level[2:0] && !pos_level[3])
                pos_taken <= pos_taken + 32'd1;

            // per CPU frame (vblank to vblank); the counts saturate at 511
            if (vbl_event) begin
                if (((pos_frame_cnt != 9'd0) && (pos_frame_cnt != 9'd224)) || (pos_ack_cnt != pos_frame_cnt))
                    if (pos_odd_frames != 16'hFFFF) pos_odd_frames <= pos_odd_frames + 16'd1;
                pos_frame     <= pos_frame_cnt;
                pos_ack_frame <= pos_ack_cnt;
                pos_frame_cnt <= {8'd0, pos_raise};
                pos_ack_cnt   <= {8'd0, wr_pos_ack};
            end else begin
                if (pos_raise  && pos_frame_cnt != 9'h1FF) pos_frame_cnt <= pos_frame_cnt + 9'd1;
                if (wr_pos_ack && pos_ack_cnt   != 9'h1FF) pos_ack_cnt   <= pos_ack_cnt + 9'd1;
            end
        end
    end
endmodule
