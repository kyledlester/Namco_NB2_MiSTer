// NB-1 M22 bench: exhaustive equivalence of the two na1_m37702 timing changes (rtl/vendor/README.md).
//  1. 16-bit decimal ADC/SBC high byte: the former S_ALU expression (carry = registered low-byte carry bl[8])
//     against the M22 selection bl[8] ? dech(..,1,sb) : dech(..,0,sb), calling the RTL's own dech() in the
//     instantiated core, for every accumulator byte, operand byte, carry and ADC/SBC (262,144 cases).
//  2. MPY Z: prod == 0 against (ra == 0 || tmp == 0) for every 8x8 pair and 2^20 16x16 pairs (all edge values
//     plus random).
`timescale 1ns/1ps
module m22_c75_alu_tb;
    reg clk = 1'b0;
    na1_m37702 dut (.clk_sys(clk), .reset(1'b1), .ce_mcu(1'b0), .bus_req(), .bus_we(), .bus_addr(), .bus_be(),
                    .bus_wdata(), .bus_ack(1'b0), .bus_rdata(16'd0), .irq_take(1'b0), .irq_line(5'd0),
                    .irq_pri(3'd0), .irq_ack(), .irq_ack_line(), .flag_i_out(), .ipl_out(), .instr_commit(),
                    .instr_start(), .instr_count(), .cycle_count(), .overrun_count(), .stopped());

    // the former S_ALU high-byte expressions (upstream na1_m37702.sv)
    function automatic [9:0] old_bh(input [7:0] a, input [7:0] s, input c, input sb);
        reg [9:0] bh;
        if (sb) begin
            bh = {2'd0,a} - {2'd0,s} - {9'd0,c};
            if (bh[3:0] > 4'd9) bh = bh - 10'd6;
            if (bh[7:4] > 4'd9) bh = bh - 10'h60;
        end else begin
            bh = {2'd0,a} + {2'd0,s} + {9'd0,c};
            if (bh[3:0] > 4'd9) bh = bh + 10'd6;
            if (bh[7:4] > 4'd9) bh = bh + 10'h60;
        end
        return bh;
    endfunction

    initial begin
        int errs = 0, n = 0;
        reg [9:0] h0, h1, sel;
        for (int sb = 0; sb < 2; sb++)
            for (int a = 0; a < 256; a++)
                for (int s = 0; s < 256; s++) begin
                    h0 = dut.dech(a[7:0], s[7:0], 1'b0, sb[0]);
                    h1 = dut.dech(a[7:0], s[7:0], 1'b1, sb[0]);
                    for (int c = 0; c < 2; c++) begin
                        sel = c ? h1 : h0;
                        n++;
                        if (sel !== old_bh(a[7:0], s[7:0], c[0], sb[0])) begin
                            errs++;
                            if (errs < 10) $display("  BCD mismatch sb %0d a %02x s %02x c %0d: %03x vs %03x", sb, a, s, c, sel, old_bh(a[7:0], s[7:0], c[0], sb[0]));
                        end
                    end
                end
        $display("  decimal high byte: %0d cases, %0d mismatches", n, errs);
        n = 0;
        for (int a = 0; a < 256; a++)
            for (int t = 0; t < 256; t++) begin
                reg [31:0] p; p = a * t; n++;
                if ((p[15:0] == 16'd0) !== (a[7:0] == 8'd0 || t[7:0] == 8'd0)) errs++;
            end
        for (int i = 0; i < (1 << 20); i++) begin
            reg [15:0] a, t; reg [31:0] p;
            a = (i < 64) ? ((i & 7) == 0 ? 16'd0 : (i & 7) == 1 ? 16'hFFFF : (i & 7) == 2 ? 16'h8000 : (i & 7) == 3 ? 16'h0100 : $urandom) : $urandom;
            t = (i < 64) ? (((i >> 3) & 7) == 0 ? 16'd0 : ((i >> 3) & 7) == 1 ? 16'hFFFF : ((i >> 3) & 7) == 2 ? 16'h8000 : ((i >> 3) & 7) == 3 ? 16'h0001 : $urandom) : $urandom;
            if (i % 5 == 4) a = 16'd0;
            p = a * t; n++;
            if ((p == 32'd0) !== (a == 16'd0 || t == 16'd0)) errs++;
        end
        $display("  MPY zero flag: %0d cases", n);
        if (errs == 0) $display("PASS M22 C75 ALU EQUIVALENCE"); else $display("FAIL M22 C75 ALU EQUIVALENCE: %0d", errs);
        $finish;
    end
endmodule
