// M20B M37702M2 on-chip peripherals for the Namco C69: special function
// registers $00-$7F, timers A0-A4/B0-B2, A-D converter, UART0/1 register
// stubs, ports P0-P8, watchdog/processor-mode stubs and the interrupt
// controller + priority resolver.
//
// Behavioural contract [MAME-CONFIRMED] = MAME 0.289 m37710.cpp
// (m37702m2_device::map, port_r/w, get_port_reg/set_port_reg, count_start_w,
// m37710_recalc_timer, m37710_timer_cb, ad_control_w, ad_timer_cb, uart*_w,
// set_int_control, m37710_set_irq_line, m37710i_update_irqs, device_reset).
// Reproduced MAME properties that differ from the datasheet (deliberately,
// see docs/M20B_IMPLEMENTATION.md sections 2-3): a started timer never stops; timer
// registers read back the written reload value; interrupt-control bit 3 is a
// register image separate from the pending line (a software write neither
// requests nor cancels); event-counter/one-shot/PWM timer modes and UART
// transfers do nothing. Physical M37702 behaviour is [UNKNOWN].
//
// Timing [IMPLEMENTATION]: MAME schedules timer/A-D starts at the time of the
// register write, which is the end of the writing instruction's cycle budget;
// the core signals that boundary with instr_commit and the starts are applied
// there. Periods count ce_mcu ticks (MAME clocks_to_attotime in input clocks).
// MAME_EARLY=1 reproduces a MAME arithmetic artefact [MAME-CONFIRMED by the
// f140 trace, docs/M20B_IMPLEMENTATION.md section 7]: 2*floor(1e18/12528250) as is one
// attosecond less than floor(1e18/6264125), so every expiry lands just below a
// CPU-cycle boundary and the scheduler's floored timeslice lets the CPU take
// the interrupt at the first instruction boundary >= (ideal expiry - 1 cycle).
// The request line is therefore asserted two ticks before the exact expiry;
// reloads and A-D result timing stay exact. MAME_EARLY=0 is the ideal model.
//
// Bus: single-cycle byte-lane slave. addr is the even byte address of the
// word ($00-$7E), be[0] = byte addr, be[1] = byte addr+1.
// NB-1 LOCAL MODIFICATION (M19, docs/M19_RESEARCH.md; rtl/vendor/README.md): COUNT_STOP=1 makes a
// count-start bit written 1 -> 0 stop that timer at the instruction boundary (M37702 datasheet: the count
// start flag = 0 stops counting). MAME never stops a started timer; COUNT_STOP=0 (default) keeps exactly
// that upstream behaviour.
module na1_c69_sfr #(parameter MAME_EARLY=1, parameter COUNT_STOP=0)(
 input wire clk_sys,reset,ce_mcu,
 input wire req,we,input wire [6:0] addr,input wire [1:0] be,input wire [15:0] wdata,
 output reg ack=0,output reg [15:0] rdata=0,
 input wire instr_commit,
 // ports: inputs as the board drives them (MAME p*_in_cb; unbound ports read $FF)
 input wire [7:0] port_in [0:8],
 output reg [7:0] port_reg [0:8],output reg [7:0] port_dir [0:8],
 output reg [8:0] port_wr=0,             // set_port_reg with dir != 0: out_cb(data & dir, dir)
 // A-D inputs (MAME analog callbacks)
 input wire [15:0] an_in [0:7],
 // external interrupt requests (one tick pulses): INT0, INT1, INT2
 input wire irq0_in,irq1_in,irq2_in,
 // resolver <-> core
 input wire flag_i,input wire [2:0] ipl,
 output reg irq_take=0,output reg [4:0] irq_line=0,output reg [2:0] irq_pri=0,
 input wire irq_ack,input wire [4:0] irq_ack_line,
 // bench injection (tied low in production): take this line at the next boundary
 input wire sim_force_valid,input wire [4:0] sim_force_line,
 input wire sim_inject_only              // bench: never take naturally pending interrupts
);
 // MAME line numbers
 localparam L_ADC=4,L_U1X=5,L_U1R=6,L_U0X=7,L_U0R=8,L_TB2=9,L_TB1=10,L_TB0=11,
            L_TA4=12,L_TA3=13,L_TA2=14,L_TA1=15,L_TA0=16,L_INT2=17,L_INT1=18,L_INT0=19;
 // ------------------------------------------------------------ registers
 reg [7:0] ad_control=0,ad_sweep=8'h03;
 reg [15:0] ad_result [0:7];
 reg [7:0] uart_mode [0:1];reg [7:0] uart_baud [0:1];reg [7:0] uart_ctrl0 [0:1];reg [7:0] uart_ctrl1 [0:1];
 reg [7:0] count_start=0,one_shot=0,up_down=0,proc_mode=0,watchdog_freq=0;
 reg [15:0] timer_reg [0:7];reg [7:0] timer_mode [0:7];
 reg [7:0] int_ctl [0:19];
 reg [19:0] pend=0;
 // timers: MAME timer objects
 reg [7:0] t_run=0;reg [25:0] t_period [0:7];reg [25:0] t_cnt [0:7];
 reg [7:0] t_start_req=0;              // count-start rising edges awaiting the instruction boundary
 reg [7:0] t_stop_req=0;               // NB-1 COUNT_STOP: count-start falling edges awaiting the boundary
 // A-D conversion timer
 reg ad_run=0;reg [10:0] ad_cnt=0;reg ad_start_req=0;reg ad_cancel_req=0;
 integer i;
 initial begin
  for(i=0;i<9;i=i+1) begin port_reg[i]=0;port_dir[i]=0;end
  for(i=0;i<8;i=i+1) begin ad_result[i]=0;timer_reg[i]=0;timer_mode[i]=0;t_period[i]=0;t_cnt[i]=0;end
  for(i=0;i<2;i=i+1) begin uart_mode[i]=0;uart_baud[i]=0;uart_ctrl0[i]=8'h08;uart_ctrl1[i]=8'h02;end
  for(i=0;i<20;i=i+1) int_ctl[i]=0;
 end
 // ------------------------------------------------------------ helpers
 // port index for byte address $02-$14 (MAME port_r<0>/<1> layout), -1 = none
 function integer port_of(input [6:0] a);
  case(a)
   7'h02: port_of=0; 7'h03: port_of=1; 7'h06: port_of=2; 7'h07: port_of=3;
   7'h0a: port_of=4; 7'h0b: port_of=5; 7'h0e: port_of=6; 7'h0f: port_of=7; 7'h12: port_of=8;
   default: port_of=-1;
  endcase
 endfunction
 function integer dir_of(input [6:0] a);
  case(a)
   7'h04: dir_of=0; 7'h05: dir_of=1; 7'h08: dir_of=2; 7'h09: dir_of=3;
   7'h0c: dir_of=4; 7'h0d: dir_of=5; 7'h10: dir_of=6; 7'h11: dir_of=7; 7'h14: dir_of=8;
   default: dir_of=-1;
  endcase
 endfunction
 // interrupt-control line for byte address $70-$7F, -1 = none
 function integer line_of(input [6:0] a);
  case(a)
   7'h70: line_of=L_ADC; 7'h71: line_of=L_U0X; 7'h72: line_of=L_U0R; 7'h73: line_of=L_U1X; 7'h74: line_of=L_U1R;
   7'h75: line_of=L_TA0; 7'h76: line_of=L_TA1; 7'h77: line_of=L_TA2; 7'h78: line_of=L_TA3; 7'h79: line_of=L_TA4;
   7'h7a: line_of=L_TB0; 7'h7b: line_of=L_TB1; 7'h7c: line_of=L_TB2; 7'h7d: line_of=L_INT0; 7'h7e: line_of=L_INT1; 7'h7f: line_of=L_INT2;
   default: line_of=-1;
  endcase
 endfunction
 // MAME get_port_reg
 function [7:0] port_read(input integer p);
  port_read=(port_dir[p]==8'hff) ? port_reg[p] : ((port_in[p]&~port_dir[p])|(port_reg[p]&port_dir[p]));
 endfunction
 // byte read of the SFR space (MAME handlers; unmapped -> 0)
 function [7:0] sfr_read(input [6:0] a);
  integer p;
  begin
   sfr_read=8'h00;
   p=port_of(a); if(p>=0) sfr_read=port_read(p);
   p=dir_of(a);  if(p>=0) sfr_read=port_dir[p];
   p=line_of(a); if(p>=0) sfr_read=int_ctl[p];
   case(a)
    7'h1e: sfr_read=ad_control;
    7'h1f: sfr_read=ad_sweep;
    7'h30: sfr_read=uart_mode[0]; 7'h34: sfr_read=uart_ctrl0[0]; 7'h35: sfr_read=uart_ctrl1[0];
    7'h38: sfr_read=uart_mode[1]; 7'h3c: sfr_read=uart_ctrl0[1]; 7'h3d: sfr_read=uart_ctrl1[1];
    7'h40: sfr_read=count_start;
    7'h44: sfr_read=up_down&8'h1f;
    7'h5e: sfr_read=proc_mode&8'hf7;
    7'h61: sfr_read=watchdog_freq;
    default: ;
   endcase
   if(a>=7'h20 && a<=7'h2f) sfr_read=a[0] ? ad_result[a[3:1]][15:8] : ad_result[a[3:1]][7:0];
   if(a>=7'h46 && a<=7'h55) sfr_read=a[0] ? timer_reg[(a-7'h46)>>1][15:8] : timer_reg[(a-7'h46)>>1][7:0];
   if(a>=7'h56 && a<=7'h5d) sfr_read=timer_mode[a-7'h56];
  end
 endfunction
 // one byte write (side effects handled by the caller where MAME has them)
 task automatic sfr_write(input [6:0] a,input [7:0] d);
  integer p;
  begin
   p=port_of(a); if(p>=0) begin port_reg[p]<=d; if(port_dir[p]!=8'h00) port_wr[p]<=1'b1; end
   p=dir_of(a);  if(p>=0) port_dir[p]<=d;
   p=line_of(a); if(p>=0) int_ctl[p]<=d;            // register image only; pending line untouched
   case(a)
    7'h1e: begin
     if(d[6] && !ad_control[6]) begin ad_start_req<=1;ad_cancel_req<=0;ad_control<=d[4] ? (d&8'hf8) : d;end
     else begin if(!d[6]) begin ad_cancel_req<=1;ad_start_req<=0;end ad_control<=d;end
    end
    7'h1f: ad_sweep<=d;
    7'h30: uart_mode[0]<=d; 7'h31: uart_baud[0]<=d;
    7'h34: uart_ctrl0[0]<=(d&~8'h08)|(uart_ctrl0[0]&8'h08);
    7'h35: uart_ctrl1[0]<=(uart_ctrl1[0]&(d[2] ? 8'hfa : 8'h0a))|(d&8'h05);
    7'h38: uart_mode[1]<=d; 7'h39: uart_baud[1]<=d;
    7'h3c: uart_ctrl0[1]<=(d&~8'h08)|(uart_ctrl0[1]&8'h08);
    7'h3d: uart_ctrl1[1]<=(uart_ctrl1[1]&(d[2] ? 8'hfa : 8'h0a))|(d&8'h05);
    7'h40: begin count_start<=d; t_start_req<=t_start_req|(d&~count_start); t_stop_req<=t_stop_req|(count_start&~d); end
    7'h42: one_shot<=d;
    7'h44: up_down<=d;
    7'h5e: proc_mode<=d;
    7'h61: watchdog_freq<=d;
    default: ;
   endcase
   if(a>=7'h46 && a<=7'h55) begin if(a[0]) timer_reg[(a-7'h46)>>1][15:8]<=d; else timer_reg[(a-7'h46)>>1][7:0]<=d; end
   if(a>=7'h56 && a<=7'h5d) timer_mode[a-7'h56]<=d;
  end
 endtask
 // MAME timer period: tscales[mode>>6] * (reg + 1) = (reg + 1) << {1,4,6,9}; 0 = no timer
 function [25:0] period_of(input integer t);
  reg [3:0] sh;
  begin
   case(timer_mode[t][7:6]) 2'd0: sh=4'd1; 2'd1: sh=4'd4; 2'd2: sh=4'd6; default: sh=4'd9; endcase
   if(timer_mode[t][1:0]!=2'd0) period_of=26'd0;                                   // not timer mode: no MAME timer
   else if(timer_reg[t]==16'd0 && timer_mode[t][7:6]==2'd0) period_of=26'd0;        // MAME's 8 MHz hack: ignored
   else period_of=({10'd0,timer_reg[t]}+26'd1)<<sh;
  end
 endfunction
 // Setup-timing fix: period_of() (mode decode, +1 and a barrel shift) is
 // registered per timer instead of being evaluated inside the load of
 // t_period/t_cnt. A timer register/mode write reaches instr_commit at least
 // three clocks later (bus ack, then the core's two-clock S_DONE), so the
 // registered copy is always current when it is used.
 reg [25:0] t_newperiod [0:7];
 integer tp;
 initial for(tp=0;tp<8;tp=tp+1) t_newperiod[tp]=0;
 always @(posedge clk_sys) for(tp=0;tp<8;tp=tp+1) t_newperiod[tp]<=period_of(tp);
 wire [10:0] ad_period=ad_control[7] ? 11'd228 : 11'd456;   // 57*2*(2|4) clocks
 // ------------------------------------------------------------ interrupt resolver (m37710i_update_irqs)
 // MAME scans the lines from 19 down to 0 and takes each one whose priority is
 // strictly above both the CPU's ipl and the best seen so far: the result is
 // the highest priority above ipl among the pending lines, and among equal
 // priorities the highest line number. Computed here in parallel form (which
 // priorities are present, then the highest line carrying the best one) instead
 // of the equivalent 20-deep sequential scan, whose serial `best` dependency
 // was a 15-level path from ipl to the core's instruction boundary (M20C
 // timing). Equivalence to the scan is checked by sim/m20c_timing_tb.sv.
 //
 // Setup-timing fix: the resolver result is registered (c_* -> irq_*), cutting
 // the ipl/flag_i -> resolver -> instruction-boundary path (-0.9 ns). The core
 // waits one extra clock at the boundary (na1_m37702 S_DONE) so the registered
 // value always reflects the ipl/I flag of the instruction that just finished.
 // NB-1 M24 (timing, local modification): the same result with ipl entering last. Per priority p,
 // pres_all[p] = some pending line has priority p, and hi_line[p] = the highest such line; both are
 // independent of ipl. Then best = the highest p > ipl with pres_all[p], and the line is hi_line[best]
 // (every line of priority best > ipl is eligible, so the highest of them is the scan's choice).
 // Equivalence with the former form: sim/m24_irq_resolver_tb.sv.
 reg [7:0] pres_all,present,top;reg [2:0] best;reg [4:0] hi_line[1:7];
 reg c_take;reg [4:0] c_line;reg [2:0] c_pri;
 always @(posedge clk_sys) begin irq_take<=c_take;irq_line<=c_line;irq_pri<=c_pri;end
 always @* begin
  pres_all=8'd0;
  for(int p=1;p<8;p=p+1) begin
   hi_line[p]=5'd0;
   for(int k=0;k<20;k=k+1) if(pend[k] && int_ctl[k][2:0]==p[2:0]) begin pres_all[p]=1'b1;hi_line[p]=k[4:0];end
  end
 end
 always @* begin
  c_take=0;c_line=0;c_pri=0;present=8'd0;top=8'd0;best=3'd0;
  if(sim_force_valid) begin c_take=1;c_line=sim_force_line;c_pri=int_ctl[sim_force_line][2:0];end
  else if(!flag_i && !sim_inject_only) begin
   for(int p=1;p<8;p=p+1) present[p]=pres_all[p] && (p[2:0]>ipl);            // above ipl (so >= 1)
   for(int p=1;p<8;p=p+1) top[p]=present[p] && !(|(present>>(p+1)));           // one-hot: best priority
   for(int p=1;p<8;p=p+1) if(top[p]) begin best=best|p[2:0];c_line=c_line|hi_line[p];end
   c_take=|present;c_pri=best;
  end
 end
 // ------------------------------------------------------------ sequential
 reg [19:0] set_line;   // lines asserted this cycle
 always @(posedge clk_sys) begin
  port_wr<=9'd0;
  set_line=20'd0;
  if(reset) begin
   ack<=0;
   for(i=0;i<9;i=i+1) port_dir[i]<=0;
   ad_control<=ad_control&8'h07;ad_sweep<=(ad_sweep&~8'hdc)|8'h03;ad_run<=0;ad_start_req<=0;ad_cancel_req<=0;
   for(i=0;i<2;i=i+1) begin uart_mode[i]<=0;uart_ctrl0[i]<=(uart_ctrl0[i]&8'he0)|8'h08;uart_ctrl1[i]<=8'h02;end
   count_start<=0;one_shot<=one_shot&~8'h1f;up_down<=0;
   for(i=0;i<8;i=i+1) timer_mode[i]<=0;
   proc_mode<=0;watchdog_freq<=watchdog_freq&~8'h01;
   for(i=0;i<=L_TA0;i=i+1) int_ctl[i]<=int_ctl[i]&~8'h0f;
   for(i=L_INT2;i<=L_INT0;i=i+1) int_ctl[i]<=int_ctl[i]&~8'h3f;
   pend<=0;t_run<=0;t_start_req<=0;t_stop_req<=0;
  end else begin
   // ---- bus
   if(!req) ack<=0;
   else if(!ack) begin
    if(we) begin
     if(be[0]) sfr_write(addr,wdata[7:0]);
     if(be[1]) sfr_write(addr|7'h01,wdata[15:8]);
    end
    rdata<={sfr_read(addr|7'h01),sfr_read(addr)};
    ack<=1;
   end
   // ---- instruction boundary: apply MAME's time-of-write side effects
   if(instr_commit) begin
    if(COUNT_STOP) for(i=0;i<8;i=i+1) if(t_stop_req[i] && !t_start_req[i]) t_run[i]<=0;
    t_stop_req<=0;
    for(i=0;i<8;i=i+1) if(t_start_req[i]) begin
     if(t_newperiod[i]!=26'd0) begin t_run[i]<=1;t_period[i]<=t_newperiod[i];t_cnt[i]<=t_newperiod[i];end
    end
    t_start_req<=0;
    if(ad_start_req) begin ad_run<=1;ad_cnt<=ad_period;end
    else if(ad_cancel_req) ad_run<=0;
    ad_start_req<=0;ad_cancel_req<=0;
   end
   // ---- timers and A-D on the MAME clock
   if(ce_mcu) begin
    for(i=0;i<8;i=i+1) if(t_run[i]) begin
     if(t_cnt[i]==(MAME_EARLY ? 26'd3 : 26'd1)) set_line[L_TA0-i]=1'b1;
     if(t_cnt[i]==26'd1) t_cnt[i]<=t_period[i]; else t_cnt[i]<=t_cnt[i]-26'd1;
    end
    if(ad_run) begin : adc
     reg [2:0] line;reg last;
     line=ad_control[2:0];
     last=!(ad_control[3] || (ad_control[4] && line!=({ad_sweep[1:0],1'b1})));
     if(ad_cnt==(MAME_EARLY ? 11'd3 : 11'd1) && last) set_line[L_ADC]=1'b1;
     if(ad_cnt==11'd1) begin
      ad_result[line]<=an_in[line];
      if(ad_control[4]) ad_control[2:0]<=line+3'd1;
      if(!last) ad_cnt<=ad_period;
      else begin ad_run<=0;ad_control[6]<=1'b0;end
     end else ad_cnt<=ad_cnt-11'd1;
    end
   end
   if(irq0_in) set_line[L_INT0]=1'b1;
   if(irq1_in) set_line[L_INT1]=1'b1;
   if(irq2_in) set_line[L_INT2]=1'b1;
   // ---- request bits: assert sets pending + image bit 3; acceptance clears both
   for(i=0;i<20;i=i+1) begin
    if(set_line[i]) begin pend[i]<=1;int_ctl[i][3]<=1'b1;end
    if(irq_ack && irq_ack_line==i[4:0]) begin pend[i]<=0;int_ctl[i][3]<=1'b0;end
   end
  end
 end
endmodule
