// M20B Mitsubishi M37702 (Namco C69) CPU core.
//
// Behavioural contract [MAME-CONFIRMED] = MAME 0.289 src/devices/cpu/m37710/
// (m37710op.h opcode macros, m37710il.h EA/stack helpers, m37710.cpp interrupt
// entry / reset). No physical M37702 behaviour has been measured; where MAME is
// known to deviate from the datasheet (see docs/M20B_IMPLEMENTATION.md section 2) the
// MAME behaviour is reproduced deliberately, because the F/A BIOS is proven to
// run on it and the differential trace compares against it.
//
// Microarchitecture [IMPLEMENTATION]: microsequenced 16-bit core on clk_sys,
// single little-endian 16-bit bus (byte lanes; a 16-bit access at an odd
// address is two byte accesses in address order), data accesses issued in
// MAME's order (push high byte first at S then S-1, pull low byte first, RMW
// read then write). Every instruction is padded to MAME's CLK cycle count in
// ce_mcu ticks (static part from na1_m37702_decode, dynamic extras added here)
// so timers/interrupts stay lock-step with MAME; bus wait states add on top.
// MAME's M37710 executes one CPU cycle per two input clocks
// (execute_clocks_to_cycles), while timer/A-D periods count input clocks; the
// core therefore spends one cycle per two ce_mcu ticks and the SFR block
// counts every tick.
//
// Bus: bus_req held with bus_we / bus_addr (byte address of the word, even) /
// bus_be / bus_wdata until bus_ack (level), then withdrawn for >= 1 cycle.
// Interrupts: irq_take/irq_line/irq_pri come from the C69 SFR block's resolver
// (m37710i_update_irqs rules); irq_ack pulses with the accepted line when the
// entry sequence starts. instr_commit pulses at each instruction boundary
// (before the interrupt decision): SFR side effects that MAME time-stamps at
// the end of the writing instruction (timer / A-D starts) are applied there.
module na1_m37702
  import na1_m37702_pkg::*;
(
 input wire clk_sys,reset,
 input wire ce_mcu,                       // MCU input clock tick (12.528 MHz); one MAME CPU cycle = 2 ticks
 output reg bus_req=0,output reg bus_we=0,output reg [23:0] bus_addr=0,
 output reg [1:0] bus_be=0,output reg [15:0] bus_wdata=0,
 input wire bus_ack,input wire [15:0] bus_rdata,
 input wire irq_take,input wire [4:0] irq_line,input wire [2:0] irq_pri,
 output reg irq_ack=0,output reg [4:0] irq_ack_line=0,
 output wire flag_i_out,output wire [2:0] ipl_out,
 output reg instr_commit=0,
 // observation (bench): high during the first fetch cycle of every instruction
 // (the registers then hold the state before it); counters
 output wire instr_start,
 output reg [31:0] instr_count=0,
 output reg [31:0] cycle_count=0,        // MAME cycles elapsed since reset (2 ce_mcu ticks each)
 output reg [15:0] overrun_count=0,
 output wire stopped
);
 // ------------------------------------------------------------ registers
 reg [15:0] ra=0,rb=0,rx=0,ry=0,rs=16'h0100,rpc=0,rdpr=0;
 reg [7:0] rpg=0,rdt=0;
 reg fn=0,fv=0,fm=0,fx=0,fd=0,fi=1,fz=1,fc=0;
 reg [2:0] ipl=0;
 assign flag_i_out=fi;assign ipl_out=ipl;
 wire [7:0] ps={fn,fv,fm,fx,fd,fi,fz,fc};
 // MAME index-register view: only the low byte participates when X=1.
 wire [15:0] xv=fx ? {8'd0,rx[7:0]} : rx;
 wire [15:0] yv=fx ? {8'd0,ry[7:0]} : ry;
 // ------------------------------------------------------------ decode / timing
 dec_t op;
 reg [1:0] pfx=0;
 reg w16=0;                 // current operation width (1 = 16 bits)
 reg [19:0] budget=0,spent=0;
 reg done_settled=0;
 reg running=0,cyc_phase=0;      // cyc_phase: second tick of the current CPU cycle
 // ------------------------------------------------------------ temporaries
 reg [23:0] ea=0,ptr=0;
 reg [15:0] tmp=0,imm=0,base=0;
 reg [16:0] cdiff=0;
 reg [7:0] mask8=0;
 reg [3:0] step=0;reg [1:0] phase=0;
 reg [31:0] prod=0;
 reg [31:0] dvd=0,quo=0;reg [16:0] rem=0;reg [15:0] dvs=0;reg [5:0] dcnt=0;
 reg sw_int=0;reg [15:0] vector=0;reg [2:0] irq_pri_l=0;
 reg halted=0;
 assign stopped=halted;
 function [15:0] vec_of(input [4:0] line);vec_of=16'hffce+{10'd0,line,1'b0};endfunction
 // ------------------------------------------------------------ memory sub-machine
 reg m_w16=0,m_we=0;reg [23:0] m_addr=0;reg [15:0] m_wdata=0,mdata=0;
 typedef enum logic [6:0] {
  S_RESET0,S_RESET1,S_FETCH,S_DECODE,
  S_EA_D,S_EA_A,S_EA_AL,S_EA_PTR16,S_EA_PTR24,S_EA_FIN,
  S_EXEC,S_OPER,S_ALU,S_RMW_W,S_SEB_M,S_SEB_W,S_BBS_M,S_BBS_R,S_BBS_GO,S_LDM_W,S_BCC_GO,
  S_JMP1,S_JMP2,S_JMP3,S_JMP4,
  S_PUSH1,S_PUSH2,S_PULL1,S_PULL2,S_PSH1,S_PSH2,S_PSH3,S_PUL1,S_PUL2,S_PUL3,
  S_MVN1,S_MVN2,S_MVN3,S_DIV1,S_DIV2,S_RLA1,S_RLA2,
  S_INT2,S_INT3,S_INT4,S_INT5,S_INT6,S_INT7,
  S_MEM1,S_MEM2,S_DONE,S_WAI,S_HALT
 } st_e;
 st_e st=S_RESET0,ret=S_FETCH;
 assign instr_start=(st==S_FETCH) && (pfx==2'd0) && !reset;
 // ------------------------------------------------------------ helpers
 function [15:0] wmask(input [15:0] v,input w);wmask=w ? v : {8'd0,v[7:0]};endfunction
 function nbit(input [15:0] v,input w);nbit=w ? v[15] : v[7];endfunction
 function zbit(input [15:0] v,input w);zbit=w ? (v==16'd0) : (v[7:0]==8'd0);endfunction
 task automatic wr_a(input [15:0] v,input w);if(w) ra<=v;else ra[7:0]<=v[7:0];endtask
 task automatic wr_b(input [15:0] v,input w);if(w) rb<=v;else rb[7:0]<=v[7:0];endtask
 task automatic wr_x(input [15:0] v,input w);if(w) rx<=v;else rx[7:0]<=v[7:0];endtask
 task automatic wr_y(input [15:0] v,input w);if(w) ry<=v;else ry[7:0]<=v[7:0];endtask
 task automatic set_nz(input [15:0] v,input w);fn<=nbit(v,w);fz<=zbit(v,w);endtask
 task automatic set_ps(input [7:0] v);
  fn<=v[7];fv<=v[6];fm<=v[5];fx<=v[4];fd<=v[3];fi<=v[2];fz<=v[1];fc<=v[0];
 endtask
 // issue a logical 8/16-bit access at any address; continue at `nxt` with mdata
 task automatic mem(input w,input we,input [23:0] a,input [15:0] d,input st_e nxt);
  m_w16=w;m_we=we;m_addr=a;m_wdata=d;ret=nxt;st=S_MEM1;
 endtask
 task automatic rd(input w,input [23:0] a,input st_e nxt);mem(w,1'b0,a,16'd0,nxt);endtask
 task automatic wr(input w,input [23:0] a,input [15:0] d,input st_e nxt);mem(w,1'b1,a,d,nxt);endtask
 task automatic fetch(input w,input st_e nxt);rd(w,{rpg,rpc},nxt);rpc=rpc+(w ? 16'd2 : 16'd1);endtask
 task automatic push8(input [7:0] v,input st_e nxt);wr(1'b0,{8'd0,rs},{8'd0,v},nxt);rs=rs-16'd1;endtask
 task automatic pull8(input st_e nxt);rs=rs+16'd1;rd(1'b0,{8'd0,rs},nxt);endtask
 task automatic start_int(input [15:0] vec,input sw,input [2:0] pri);
  vector<=vec;sw_int<=sw;irq_pri_l<=pri;push8(rpg,S_INT2);
 endtask
 function wsel(input dec_t d);
  case(d.wsrc) 2'd0: wsel=!fm; 2'd1: wsel=!fx; 2'd2: wsel=1'b0; default: wsel=1'b1; endcase
 endfunction
 // ------------------------------------------------------------ ALU (MAME OP_* semantics)
 // Decimal ADC/SBC low byte (MAME's nibble rule: correct the low nibble of the
 // 8-bit sum when > 9, then the high nibble of the corrected value when > 9),
 // computed from the operand register mdata and registered at S_OPER, the state
 // that precedes S_ALU for every C_ALU instruction; S_ALU then does only the
 // high byte. Same values as the former single-cycle chain of six adders (M20C
 // timing cut; equivalence checked by sim/m20c_timing_tb.sv). S_OPER changes
 // none of acc/fc/fd/w16, and tmp (used by S_ALU) is mdata, so the registered
 // low byte is exactly what S_ALU computed before.
 reg [9:0] dl_raw,dl;reg [9:0] dbl_raw=0,dbl=0;
 wire [7:0] dacc=op.useb ? rb[7:0] : ra[7:0];
 always @* begin
  if(op.sub==4'd6) begin
   dl_raw={2'd0,dacc}-{2'd0,mdata[7:0]}-{9'd0,~fc};dl=dl_raw;
   if(dl[3:0]>4'd9) dl=dl-10'd6;
   if(dl[7:4]>4'd9) dl=dl-10'h60;
  end else begin
   dl_raw={2'd0,dacc}+{2'd0,mdata[7:0]}+{9'd0,fc};dl=dl_raw;
   if(dl[3:0]>4'd9) dl=dl+10'd6;
   if(dl[7:4]>4'd9) dl=dl+10'h60;
  end
 end
 // NB-1 M22 (timing, local modification): the 16-bit decimal high byte likewise, for both possible
 // low-byte carries/borrows (0 and 1), registered at S_OPER in parallel with dbl; S_ALU selects with
 // dbl[8]. dech() is the former S_ALU high-byte expression with the carry as an argument, so the
 // selected value is exactly the former one (acc and mdata = tmp are unchanged between S_OPER and
 // S_ALU; exhaustive check: sim/m22_c75_alu_tb.sv).
 function [9:0] dech(input [7:0] a,input [7:0] s,input c,input sb);
  reg [9:0] h;
  begin
   if(sb) begin
    h={2'd0,a}-{2'd0,s}-{9'd0,c};
    if(h[3:0]>4'd9) h=h-10'd6;
    if(h[7:4]>4'd9) h=h-10'h60;
   end else begin
    h={2'd0,a}+{2'd0,s}+{9'd0,c};
    if(h[3:0]>4'd9) h=h+10'd6;
    if(h[7:4]>4'd9) h=h+10'h60;
   end
   dech=h;
  end
 endfunction
 wire [7:0] dacch=op.useb ? rb[15:8] : ra[15:8];
 reg [9:0] dbh0=0,dbh1=0;
 reg [15:0] acc,src,alu_r;reg [16:0] sum;reg alu_n,alu_v,alu_z,alu_c,alu_wf;
 reg [9:0] bl,bh;
 always @* begin
  acc=op.useb ? rb : ra;
  if(!w16) acc={8'd0,acc[7:0]};
  src=w16 ? tmp : {8'd0,tmp[7:0]};
  alu_r=acc;alu_n=fn;alu_v=fv;alu_z=fz;alu_c=fc;alu_wf=0;sum=17'd0;bl=10'd0;bh=10'd0;
  case(op.sub)
   4'd0: begin alu_r=acc|src;alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);end            // ORA
   4'd1: begin alu_r=acc&src;alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);end            // AND
   4'd2: begin alu_r=acc^src;alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);end            // EOR
   4'd3: begin                                                                          // ADC
    alu_wf=1;
    if(!fd) begin
     sum={1'b0,acc}+{1'b0,src}+{16'd0,fc};
     alu_r=wmask(sum[15:0],w16);
     alu_c=w16 ? sum[16] : sum[8];
     alu_v=w16 ? ((src[15]^sum[15])&(acc[15]^sum[15])) : ((src[7]^sum[7])&(acc[7]^sum[7]));
     alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);
    end else if(!w16) begin
     bl=dbl;
     alu_r={8'd0,bl[7:0]};alu_c=bl[8];
     alu_v=(src[7]^bl[7])&(acc[7]^bl[7]);
     alu_n=bl[7];alu_z=(bl[7:0]==8'd0);
    end else begin
     bl=dbl;
     bh=bl[8] ? dbh1 : dbh0;           // M22: = dech(acc[15:8],src[15:8],bl[8],0)
     alu_r={bh[7:0],bl[7:0]};alu_c=bh[8];
     alu_n=bh[7];alu_z=(alu_r==16'd0);
     // MAME VFLAG_ADD_16(SRC, REG_A, FLAG_C) with FLAG_C holding only the
     // high-byte partial sum: bit 15 of (src^bh)&(acc^bh) = src[15]&acc[15]
     alu_v=src[15]&acc[15];
    end
   end
   4'd4: begin alu_r=src;alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);end                // LDA
   4'd5: begin                                                                          // CMP
    sum={1'b0,acc}-{1'b0,src};
    alu_n=nbit(sum[15:0],w16);alu_z=zbit(sum[15:0],w16);
    alu_c=w16 ? ~sum[16] : ~sum[8];
   end
   4'd6: begin                                                                          // SBC
    alu_wf=1;
    if(!fd) begin
     sum={1'b0,acc}-{1'b0,src}-{16'd0,~fc};
     alu_r=wmask(sum[15:0],w16);
     alu_c=w16 ? ~sum[16] : ~sum[8];
     alu_v=w16 ? ((src[15]^acc[15])&(sum[15]^acc[15])) : ((src[7]^acc[7])&(sum[7]^acc[7]));
     alu_n=nbit(alu_r,w16);alu_z=zbit(alu_r,w16);
    end else if(!w16) begin
     alu_v=(src[7]^acc[7])&(dbl_raw[7]^acc[7]);     // V from the uncorrected difference
     bl=dbl;
     alu_r={8'd0,bl[7:0]};alu_c=~bl[8];
     alu_n=bl[7];alu_z=(bl[7:0]==8'd0);
    end else begin
     bl=dbl;
     bh=bl[8] ? dbh1 : dbh0;           // M22: = dech(acc[15:8],src[15:8],bl[8],1)
     alu_r={bh[7:0],bl[7:0]};alu_c=~bh[8];
     alu_n=bh[7];alu_z=(alu_r==16'd0);
     alu_v=(src[15]^acc[15])&(alu_r[15]^acc[15]);
    end
   end
   default: ;
  endcase
 end
 // shifter for RMW / RMWA (sub: 0 ASL 1 ROL 2 LSR 3 ROR 4 INC 5 DEC) -> {carry, result}
 function [16:0] shift(input [3:0] s,input [15:0] v,input w,input c);
  reg [15:0] r;reg co;
  begin
   case(s)
    4'd0: begin r=v<<1;co=w ? v[15] : v[7];end
    4'd1: begin r=(v<<1)|{15'd0,c};co=w ? v[15] : v[7];end
    4'd2: begin r=w ? (v>>1) : {8'd0,v[7:1]};co=v[0];end
    4'd3: begin r=w ? {c,v[15:1]} : {8'd0,c,v[7:1]};co=v[0];end
    4'd4: begin r=v+16'd1;co=c;end
    default: begin r=v-16'd1;co=c;end
   endcase
   shift={co,wmask(r,w)};
  end
 endfunction
 // branch condition (MAME COND_*; fz=1 means zero)
 function cond(input [3:0] s);
  case(s) 4'd0: cond=!fn; 4'd1: cond=fn; 4'd2: cond=!fv; 4'd3: cond=fv;
          4'd4: cond=!fc; 4'd5: cond=fc; 4'd6: cond=!fz; default: cond=fz; endcase
 endfunction
 // MAME's page-cross test for AX, AY and DIY: (base ^ (base + X)) & 0xff00
 wire [15:0] pc_sum=base+xv;
 wire page_cross=|((base^pc_sum)&16'hff00);
 wire [16:0] shl=shift(op.sub,op.useb ? rb : ra,w16,fc);   // accumulator shifts
 wire [16:0] shm=shift(op.sub,mdata,w16,fc);                // memory RMW on the read data
 wire [15:0] xreg=op.sub[0] ? ry : rx;
 // PSH/PUL register list helpers: bit -> {value, is16, cost}
 // ------------------------------------------------------------ main sequencer
 always @(posedge clk_sys) begin
  irq_ack<=0;instr_commit<=0;
  if(ce_mcu && running) begin cyc_phase<=~cyc_phase;if(cyc_phase) begin spent<=spent+20'd1;cycle_count<=cycle_count+32'd1;end end
  if(reset) begin
   st<=S_RESET0;bus_req<=0;bus_we<=0;bus_be<=0;bus_addr<=0;bus_wdata<=0;
   ra<=0;rb<=0;rx<=0;ry<=0;rs<=16'h0100;rpc<=0;rdpr<=0;rpg<=0;rdt<=0;
   fn<=0;fv<=0;fm<=0;fx<=0;fd<=0;fi<=1;fz<=1;fc<=0;ipl<=0;
   pfx<=0;budget<=0;spent<=0;running<=0;cyc_phase<=0;halted<=0;sw_int<=0;done_settled<=0;
   instr_count<=0;overrun_count<=0;cycle_count<=0;
  end else case(st)
   // ---------------------------------------------------------- reset vector
   S_RESET0: rd(1'b1,24'h00fffe,S_RESET1);
   S_RESET1: begin rpc<=mdata;pfx<=0;budget<=0;spent<=0;running<=1;st<=S_FETCH;end
   // ---------------------------------------------------------- fetch / decode
   S_FETCH: begin
    if(pfx==2'd0) instr_count<=instr_count+32'd1;
    fetch(1'b0,S_DECODE);
   end
   S_DECODE: begin
    op=decode(pfx,mdata[7:0]);
    if(op.cls==C_PFB) begin pfx<=2'd1;budget<=budget+20'd2;st<=S_FETCH;end
    else if(op.cls==C_PFXM) begin pfx<=2'd2;st<=S_FETCH;end
    else begin
     w16=wsel(op);
     budget<=budget+(w16 ? {14'd0,op.c16} : {14'd0,op.c8});
     step<=0;phase<=0;
     case(op.mode)
      M_NONE,M_IMM: st<=S_EXEC;
      M_A,M_AX,M_AY: fetch(1'b1,S_EA_A);
      M_AL,M_ALX: fetch(1'b1,S_EA_AL);
      default: fetch(1'b0,S_EA_D);      // direct-page and stack-relative families
     endcase
    end
   end
   // ---------------------------------------------------------- effective address
   S_EA_D: begin  // mdata = direct-page / stack offset byte
    case(op.mode)
     M_D,M_DI,M_DLI,M_DIY,M_DLIY: begin
      if(rdpr[7:0]!=8'd0) budget<=budget+20'd1;          // EA_D: +1 when DPR[7:0] != 0
      ptr<={8'd0,rdpr+{8'd0,mdata[7:0]}};
     end
     M_DX,M_DXI: ptr<={8'd0,rdpr+{8'd0,mdata[7:0]}+xv};
     M_DY: ptr<={8'd0,rdpr+{8'd0,mdata[7:0]}+yv};
     default: ptr<={8'd0,rs+{8'd0,mdata[7:0]}};           // S, SIY
    endcase
    st<=S_EA_FIN;
   end
   S_EA_A: begin base<=mdata;ptr<={rdt,mdata};st<=S_EA_FIN;end
   S_EA_AL: begin imm<=mdata;fetch(1'b0,S_EA_PTR24);end
   S_EA_PTR24: begin ptr<={mdata[7:0],imm};st<=S_EA_FIN;step<=4'd1;end   // AL/ALX bank byte, or DLI/DLIY third byte
   S_EA_PTR16: begin  // 16-bit pointer (DI/DXI/DIY/SIY) or the low 16 of DLI/DLIY
    imm<=mdata;
    if(op.mode==M_DLI || op.mode==M_DLIY) rd(1'b0,ptr+24'd2,S_EA_PTR24);
    else begin base<=mdata;ptr<={rdt,mdata};st<=S_EA_FIN;step<=4'd1;end
   end
   S_EA_FIN: begin
    case(op.mode)
     M_AX: begin if(page_cross) budget<=budget+20'd1;ea<=ptr+{8'd0,xv};st<=S_EXEC;end
     M_AY: begin if(page_cross) budget<=budget+20'd1;ea<=ptr+{8'd0,yv};st<=S_EXEC;end   // MAME tests the page with X
     M_ALX: begin ea<=ptr+{8'd0,xv};st<=S_EXEC;end
     M_DI,M_DXI,M_DLI: begin if(step==4'd0) rd(1'b1,ptr,S_EA_PTR16); else begin ea<=ptr;st<=S_EXEC;end end
     M_DIY: begin
      if(step==4'd0) rd(1'b1,ptr,S_EA_PTR16);
      else begin if(page_cross) budget<=budget+20'd1;ea<=ptr+{8'd0,yv};st<=S_EXEC;end
     end
     M_DLIY: begin if(step==4'd0) rd(1'b1,ptr,S_EA_PTR16); else begin ea<=ptr+{8'd0,yv};st<=S_EXEC;end end
     M_SIY: begin if(step==4'd0) rd(1'b1,ptr,S_EA_PTR16); else begin ea<={rdt,imm+yv};st<=S_EXEC;end end
     default: begin ea<=ptr;st<=S_EXEC;end                                    // D, DX, DY, S, A, AL
    endcase
   end
   // ---------------------------------------------------------- execute
   S_EXEC: begin
    case(op.cls)
     C_ALU,C_LDX,C_CPX,C_MPY,C_DIV: begin if(op.mode==M_IMM) fetch(w16,S_OPER); else rd(w16,ea,S_OPER);end
     C_STA: wr(w16,ea,op.useb ? rb : ra,S_DONE);
     C_STX: wr(w16,ea,xreg,S_DONE);
     C_RMW: rd(w16,ea,S_RMW_W);
     C_RMWA: begin
      if(op.useb) wr_b(shl[15:0],w16); else wr_a(shl[15:0],w16);
      set_nz(shl[15:0],w16);if(op.sub<4'd4) fc<=shl[16];
      st<=S_DONE;
     end
     C_INCX: begin
      tmp=xreg+(op.sub[1] ? 16'hffff : 16'h0001);
      if(op.sub[0]) wr_y(tmp,w16); else wr_x(tmp,w16);
      set_nz(tmp,w16);st<=S_DONE;
     end
     C_SEB: rd(w16,ea,S_SEB_M);
     C_BBS: rd(w16,ea,S_BBS_M);
     C_LDM: fetch(w16,S_LDM_W);
     C_BCC: fetch(op.sub==4'd9,S_BCC_GO);
     C_JMP: begin
      case(op.sub)
       4'd8,4'd9,4'd10: pull8(S_JMP2);                        // RTS / RTL / RTI
       default: fetch(1'b1,S_JMP1);                           // 16-bit operand (24-bit: bank follows)
      endcase
     end
     C_PUSH: begin
      case(op.sub)
       4'd0: begin tmp=op.useb ? rb : ra; if(w16) push8(tmp[15:8],S_PUSH2); else push8(tmp[7:0],S_DONE);end
       4'd1: begin tmp=rx; if(w16) push8(tmp[15:8],S_PUSH2); else push8(tmp[7:0],S_DONE);end
       4'd2: begin tmp=ry; if(w16) push8(tmp[15:8],S_PUSH2); else push8(tmp[7:0],S_DONE);end
       4'd3: push8({5'd0,ipl},S_PUSH1);                                   // PHP: ipl then ps
       4'd4: begin tmp=rdpr;push8(tmp[15:8],S_PUSH2);end                  // PHD
       4'd5: push8(rdt,S_DONE);                                           // PHT
       4'd6: push8(rpg,S_DONE);                                           // PHK
       4'd7: fetch(1'b1,S_PUSH1);                                         // PEA
       4'd8: begin tmp=ea[15:0];push8(tmp[15:8],S_PUSH2);end              // PEI (EA_DI done)
       default: fetch(1'b1,S_PUSH1);                                      // PER
      endcase
     end
     C_PULL: pull8(S_PULL1);
     C_PSH: fetch(1'b0,S_PSH1);
     C_PUL: fetch(1'b0,S_PUL1);
     C_FLAG: begin
      case(op.sub)
       4'd0: begin fc<=0;st<=S_DONE;end
       4'd1: begin fc<=1;st<=S_DONE;end
       4'd2: begin fi<=0;st<=S_DONE;end
       4'd3: begin fi<=1;st<=S_DONE;end
       4'd4: begin fv<=0;st<=S_DONE;end
       4'd5: begin fm<=0;st<=S_DONE;end
       4'd6: begin fm<=1;st<=S_DONE;end
       default: fetch(1'b0,S_OPER);        // REP / SEP
      endcase
     end
     C_TRANS: begin
      case(op.sub)
       4'd0,4'd1: begin  // TAX/TAY (TBX/TBY): X8 -> low byte; X16 -> the full 16-bit accumulator
        tmp=op.useb ? rb : ra;
        if(op.sub[0]) wr_y(tmp,!fx); else wr_x(tmp,!fx);
        set_nz(tmp,!fx);
       end
       4'd2,4'd3: begin  // TXA/TYA (TXB/TYB): source is MAME's index view (8-bit value when X=1)
        tmp=op.sub[0] ? yv : xv;
        if(op.useb) wr_b(tmp,!fm); else wr_a(tmp,!fm);
        set_nz(tmp,!fm);
       end
       4'd4: begin wr_x(rs,!fx);set_nz(rs,!fx);end                          // TSX
       4'd5: rs<=xv;                                                        // TXS (X8 clears S[15:8])
       4'd6: begin wr_y(rx,!fx);set_nz(rx,!fx);end                          // TXY
       4'd7: begin wr_x(ry,!fx);set_nz(ry,!fx);end                          // TYX
       4'd8: rs<=op.useb ? rb : ra;                                         // TAS/TBS: full 16 bits
       4'd9: begin if(op.useb) rb<=rs; else ra<=rs; set_nz(rs,1'b1);end     // TSA/TSB: all 16 bits
       4'd10: rdpr<=op.useb ? rb : ra;                                      // TAD/TBD
       default: begin if(op.useb) rb<=rdpr; else ra<=rdpr; set_nz(rdpr,1'b1);end // TDA/TDB
      endcase
      st<=S_DONE;
     end
     C_XAB: begin
      if(w16) begin ra<=rb;rb<=ra;set_nz(rb,1'b1);end
      else begin ra[7:0]<=rb[7:0];rb[7:0]<=ra[7:0];set_nz(rb,1'b0);end
      st<=S_DONE;
     end
     C_NOP,C_UNIMP: st<=S_DONE;
     C_WAI: st<=S_WAI;
     C_STP: begin halted<=1;st<=S_HALT;end
     C_BRK: begin rpc<=rpc+16'd1;budget<=budget+20'd13;start_int(16'hfffa,1'b1,ipl);end
     C_MVN: fetch(1'b0,S_MVN1);
     C_RLA: fetch(w16,S_RLA1);
     C_LDT: fetch(1'b0,S_OPER);
     default: st<=S_DONE;
    endcase
   end
   // operand arrived (mdata) for ALU-type classes, REP/SEP, LDT
   S_OPER: begin
    tmp=mdata;dbl_raw<=dl_raw;dbl<=dl;
    dbh0<=dech(dacch,mdata[15:8],1'b0,op.sub==4'd6);dbh1<=dech(dacch,mdata[15:8],1'b1,op.sub==4'd6);   // M22
    case(op.cls)
     C_ALU: st<=S_ALU;
     C_LDX: begin if(op.sub[0]) wr_y(tmp,w16); else wr_x(tmp,w16); set_nz(tmp,w16);st<=S_DONE;end
     C_CPX: begin
      cdiff={1'b0,wmask(xreg,w16)}-{1'b0,wmask(tmp,w16)};
      set_nz(cdiff[15:0],w16);fc<=w16 ? ~cdiff[16] : ~cdiff[8];st<=S_DONE;
     end
     C_MPY: begin
      prod=w16 ? (ra*tmp) : ({8'd0,ra[7:0]}*{8'd0,tmp[7:0]});
      // M22 (timing, local modification): Z from the operands, not through the multiplier. The product
      // is never truncated (16x16 -> 32, 8x8 -> 16), so it is zero exactly when a factor is zero.
      if(w16) begin ra<=prod[15:0];rb<=prod[31:16];fz<=(ra==16'd0)||(tmp==16'd0);end
      else begin ra[7:0]<=prod[7:0];rb[7:0]<=prod[15:8];fz<=(ra[7:0]==8'd0)||(tmp[7:0]==8'd0);end
      fn<=0;fc<=0;                      // MAME stores N as 0/1 and tests it with &0x80: always clear
      st<=S_DONE;
     end
     C_DIV: begin
      dvs=w16 ? tmp : {8'd0,tmp[7:0]};
      dvd<=w16 ? {rb,ra} : {16'd0,rb[7:0],ra[7:0]};
      quo<=0;rem<=0;dcnt<=6'd0;
      if(dvs==16'd0) begin budget<=budget+20'd13;start_int(16'hfffc,1'b1,ipl);end
      else begin budget<=budget+(w16 ? 20'd23 : 20'd8);st<=S_DIV1;end
     end
     C_FLAG: begin if(op.sub==4'd7) set_ps(ps&~tmp[7:0]); else set_ps(ps|tmp[7:0]); st<=S_DONE;end
     C_LDT: begin rdt<=tmp[7:0];st<=S_DONE;end
     default: st<=S_DONE;
    endcase
   end
   S_ALU: begin
    if(op.sub!=4'd5) begin if(op.useb) wr_b(alu_r,w16); else wr_a(alu_r,w16);end
    fn<=alu_n;fz<=alu_z;
    if(op.sub==4'd5) fc<=alu_c;
    if(alu_wf) begin fc<=alu_c;fv<=alu_v;end
    st<=S_DONE;
   end
   S_RMW_W: begin set_nz(shm[15:0],w16);if(op.sub<4'd4) fc<=shm[16];wr(w16,ea,shm[15:0],S_DONE);end
   S_SEB_M: begin tmp<=mdata;fetch(w16,S_SEB_W);end
   S_SEB_W: begin
    if(op.sub==4'd0) wr(w16,ea,tmp|wmask(mdata,w16),S_DONE); else wr(w16,ea,tmp&~wmask(mdata,w16),S_DONE);
   end
   S_BBS_M: begin tmp<=mdata;fetch(w16,S_BBS_R);end
   S_BBS_R: begin imm<=wmask(mdata,w16);fetch(1'b0,S_BBS_GO);end
   S_BBS_GO: begin
    if(op.sub==4'd0 ? ((wmask(tmp,w16)&imm)==imm) : ((wmask(tmp,w16)&imm)==16'd0)) begin
     budget<=budget+20'd3;rpc<=rpc+{{8{mdata[7]}},mdata[7:0]};
    end
    st<=S_DONE;
   end
   S_LDM_W: wr(w16,ea,mdata,S_DONE);
   S_BCC_GO: begin
    if(op.sub==4'd9) rpc<=rpc+mdata;                                  // BRL
    else if(op.sub==4'd8 || cond(op.sub)) begin
     if(op.sub!=4'd8) budget<=budget+20'd1;
     rpc<=rpc+{{8{mdata[7]}},mdata[7:0]};
    end
    st<=S_DONE;
   end
   // ---------------------------------------------------------- jumps / calls / returns
   S_JMP1: begin  // mdata = 16-bit operand
    base<=mdata;imm<=mdata;
    case(op.sub)
     4'd0: begin rpc<=mdata;st<=S_DONE;end                                   // JMP a
     4'd1,4'd4: rd(1'b1,{8'd0,mdata},S_JMP3);                                // JMP (a) / JML (a): bank-0 pointer
     4'd2,4'd6: rd(1'b1,{rpg,mdata+xv},S_JMP3);                              // JMP (a,X) / JSR (a,X): program bank
     4'd3,4'd7: fetch(1'b0,S_JMP4);                                          // JMP al / JSL: bank byte
     default: push8(rpc[15:8],S_JMP4);                                       // JSR a: push the next PC
    endcase
   end
   S_JMP3: begin  // pointer word arrived
    case(op.sub)
     4'd1,4'd2: begin rpc<=mdata;st<=S_DONE;end
     4'd4: begin imm<=mdata;rd(1'b0,{8'd0,base+16'd2},S_JMP4);end            // JML (a): third pointer byte
     default: begin imm<=mdata;push8(rpc[15:8],S_JMP4);end                   // JSR (a,X)
    endcase
   end
   S_JMP4: begin
    case(op.sub)
     4'd3,4'd4: begin rpg<=mdata[7:0];rpc<=imm;st<=S_DONE;end                // JMP al / JML (a)
     4'd5,4'd6: begin if(step==4'd0) begin step<=4'd1;push8(rpc[7:0],S_JMP4);end else begin rpc<=imm;st<=S_DONE;end end
     default: begin  // JSL: bank in mdata; push PG, PCH, PCL; jump
      case(step)
       4'd0: begin step<=4'd1;ptr<={mdata[7:0],imm};push8(rpg,S_JMP4);end
       4'd1: begin step<=4'd2;push8(rpc[15:8],S_JMP4);end
       4'd2: begin step<=4'd3;push8(rpc[7:0],S_JMP4);end
       default: begin rpg<=ptr[23:16];rpc<=ptr[15:0];st<=S_DONE;end
      endcase
     end
    endcase
   end
   S_JMP2: begin  // pull sequences: RTS (lo,hi) RTL (lo,hi,pg) RTI (ps,ipl,lo,hi,pg)
    case(op.sub)
     4'd8: begin if(step==4'd0) begin imm[7:0]<=mdata[7:0];step<=4'd1;pull8(S_JMP2);end
                 else begin rpc<={mdata[7:0],imm[7:0]};st<=S_DONE;end end
     4'd9: begin
      case(step)
       4'd0: begin imm[7:0]<=mdata[7:0];step<=4'd1;pull8(S_JMP2);end
       4'd1: begin imm[15:8]<=mdata[7:0];step<=4'd2;pull8(S_JMP2);end
       default: begin rpc<=imm;rpg<=mdata[7:0];st<=S_DONE;end
      endcase
     end
     default: begin
      case(step)
       4'd0: begin set_ps(mdata[7:0]);step<=4'd1;pull8(S_JMP2);end
       4'd1: begin ipl<=mdata[2:0];step<=4'd2;pull8(S_JMP2);end
       4'd2: begin imm[7:0]<=mdata[7:0];step<=4'd3;pull8(S_JMP2);end
       4'd3: begin imm[15:8]<=mdata[7:0];step<=4'd4;pull8(S_JMP2);end
       default: begin rpc<=imm;rpg<=mdata[7:0];st<=S_DONE;end
      endcase
     end
    endcase
   end
   // ---------------------------------------------------------- pushes / pulls
   S_PUSH1: begin  // PHP second byte; PEA/PER operand arrived
    case(op.sub)
     4'd3: push8(ps,S_DONE);
     4'd7: begin tmp=mdata;push8(tmp[15:8],S_PUSH2);end                          // PEA
     default: begin tmp=rpc+mdata;push8(tmp[15:8],S_PUSH2);end                   // PER: PC after operand + disp
    endcase
   end
   S_PUSH2: push8(tmp[7:0],S_DONE);
   S_PULL1: begin  // first pulled byte
    case(op.sub)
     4'd0: begin if(w16) begin imm[7:0]<=mdata[7:0];pull8(S_PULL2);end
                 else begin if(op.useb) rb[7:0]<=mdata[7:0]; else ra[7:0]<=mdata[7:0]; set_nz(mdata,1'b0);st<=S_DONE;end end
     4'd1: begin if(w16) begin imm[7:0]<=mdata[7:0];pull8(S_PULL2);end else begin rx[7:0]<=mdata[7:0];set_nz(mdata,1'b0);st<=S_DONE;end end
     4'd2: begin if(w16) begin imm[7:0]<=mdata[7:0];pull8(S_PULL2);end else begin ry[7:0]<=mdata[7:0];set_nz(mdata,1'b0);st<=S_DONE;end end
     4'd3: begin set_ps(mdata[7:0]);pull8(S_PULL2);end                           // PLP: ps then ipl
     4'd4: begin imm[7:0]<=mdata[7:0];pull8(S_PULL2);end                         // PLD
     default: begin rdt<=mdata[7:0];set_nz(mdata,1'b0);st<=S_DONE;end            // PLT
    endcase
   end
   S_PULL2: begin
    tmp={mdata[7:0],imm[7:0]};
    case(op.sub)
     4'd0: begin if(op.useb) rb<=tmp; else ra<=tmp; set_nz(tmp,1'b1);end
     4'd1: begin rx<=tmp;set_nz(tmp,1'b1);end
     4'd2: begin ry<=tmp;set_nz(tmp,1'b1);end
     4'd3: ipl<=mdata[2:0];
     default: rdpr<=tmp;
    endcase
    st<=S_DONE;
   end
   // PSH: mask in mdata; bits 0..7 in order = A B X Y DPR DT PG PS(ipl,ps)
   // NB-1 M24 (timing, local modification): mask8 is kept in visit order and shifted right each time
   // step advances by one, so the bit for the current step is always mask8[0] (was mask8[step] / a
   // fixed per-step bit for PUL): same decisions, same states; the 8:1 select left the m_addr path.
   S_PSH1: begin mask8<=mdata[7:0];step<=0;st<=S_PSH2;end
   S_PSH2: begin
    if(step==4'd8) st<=S_DONE;
    else if(!mask8[0]) begin step<=step+4'd1;mask8<=mask8>>1;end
    else begin
     case(step[2:0])
      3'd0: begin tmp=ra;budget<=budget+20'd2;if(!fm) push8(ra[15:8],S_PSH3); else begin push8(ra[7:0],S_PSH2);step<=4'd1;mask8<=mask8>>1;end end
      3'd1: begin tmp=rb;budget<=budget+20'd2;if(!fm) push8(rb[15:8],S_PSH3); else begin push8(rb[7:0],S_PSH2);step<=4'd2;mask8<=mask8>>1;end end
      3'd2: begin tmp=rx;budget<=budget+20'd2;if(!fx) push8(rx[15:8],S_PSH3); else begin push8(rx[7:0],S_PSH2);step<=4'd3;mask8<=mask8>>1;end end
      3'd3: begin tmp=ry;budget<=budget+20'd2;if(!fx) push8(ry[15:8],S_PSH3); else begin push8(ry[7:0],S_PSH2);step<=4'd4;mask8<=mask8>>1;end end
      3'd4: begin tmp=rdpr;budget<=budget+20'd2;push8(rdpr[15:8],S_PSH3);end
      3'd5: begin budget<=budget+20'd1;push8(rdt,S_PSH2);step<=4'd6;mask8<=mask8>>1;end
      3'd6: begin budget<=budget+20'd1;push8(rpg,S_PSH2);step<=4'd7;mask8<=mask8>>1;end
      default: begin tmp={8'd0,ps};budget<=budget+20'd2;push8({5'd0,ipl},S_PSH3);end
     endcase
    end
   end
   S_PSH3: begin push8(tmp[7:0],S_PSH2);step<=step+4'd1;mask8<=mask8>>1;end     // second (low) byte
   // PUL: mask in mdata; order PS(ps,ipl) DT DPR Y X B A; widths from the new
   // flags; bit 6 has no effect. step = list index, phase = byte within it.
   S_PUL1: begin mask8<={1'b0,mdata[0],mdata[1],mdata[2],mdata[3],mdata[4],mdata[5],mdata[7]};step<=0;phase<=0;st<=S_PUL2;end   // visit order
   S_PUL2: begin
    case(step)
     4'd0: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else begin step<=4'd1;mask8<=mask8>>1;end end
     4'd1: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else begin step<=4'd2;mask8<=mask8>>1;end end
     4'd2: begin if(mask8[0]) begin budget<=budget+20'd4;pull8(S_PUL3);end else begin step<=4'd3;mask8<=mask8>>1;end end
     4'd3: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else begin step<=4'd4;mask8<=mask8>>1;end end
     4'd4: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else begin step<=4'd5;mask8<=mask8>>1;end end
     4'd5: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else begin step<=4'd6;mask8<=mask8>>1;end end
     4'd6: begin if(mask8[0]) begin budget<=budget+20'd3;pull8(S_PUL3);end else st<=S_DONE;end
     default: st<=S_DONE;
    endcase
   end
   S_PUL3: begin  // a pulled byte arrived for list entry `step`, byte `phase`
    phase<=0;
    case(step)
     4'd0: begin if(phase==2'd0) begin set_ps(mdata[7:0]);phase<=2'd1;pull8(S_PUL3);end else begin ipl<=mdata[2:0];step<=4'd1;mask8<=mask8>>1;st<=S_PUL2;end end
     4'd1: begin rdt<=mdata[7:0];step<=4'd2;mask8<=mask8>>1;st<=S_PUL2;end
     4'd2: begin if(phase==2'd0) begin rdpr[7:0]<=mdata[7:0];phase<=2'd1;pull8(S_PUL3);end else begin rdpr[15:8]<=mdata[7:0];step<=4'd3;mask8<=mask8>>1;st<=S_PUL2;end end
     4'd3: begin if(phase==2'd0) begin ry[7:0]<=mdata[7:0];if(fx) begin step<=4'd4;mask8<=mask8>>1;st<=S_PUL2;end else begin phase<=2'd1;pull8(S_PUL3);end end
                 else begin ry[15:8]<=mdata[7:0];step<=4'd4;mask8<=mask8>>1;st<=S_PUL2;end end
     4'd4: begin if(phase==2'd0) begin rx[7:0]<=mdata[7:0];if(fx) begin step<=4'd5;mask8<=mask8>>1;st<=S_PUL2;end else begin phase<=2'd1;pull8(S_PUL3);end end
                 else begin rx[15:8]<=mdata[7:0];step<=4'd5;mask8<=mask8>>1;st<=S_PUL2;end end
     4'd5: begin if(phase==2'd0) begin rb[7:0]<=mdata[7:0];if(fm) begin step<=4'd6;mask8<=mask8>>1;st<=S_PUL2;end else begin phase<=2'd1;pull8(S_PUL3);end end
                 else begin rb[15:8]<=mdata[7:0];step<=4'd6;mask8<=mask8>>1;st<=S_PUL2;end end
     default: begin if(phase==2'd0) begin ra[7:0]<=mdata[7:0];if(fm) st<=S_DONE; else begin phase<=2'd1;pull8(S_PUL3);end end
                 else begin ra[15:8]<=mdata[7:0];st<=S_DONE;end end
    endcase
   end
   // ---------------------------------------------------------- block move (one byte per pass)
   S_MVN1: begin ptr[23:16]<=mdata[7:0];fetch(1'b0,S_MVN2);end               // destination bank
   S_MVN2: begin
    rdt<=ptr[23:16];                                                          // REG_DT = DST (even when A == 0)
    if(ra!=16'd0) rd(1'b0,{mdata[7:0],xv},S_MVN3); else st<=S_DONE;
   end
   S_MVN3: begin
    wr(1'b0,{ptr[23:16],yv},mdata,S_DONE);
    if(op.sub==4'd0) begin wr_x(rx+16'd1,!fx);wr_y(ry+16'd1,!fx);end
    else begin wr_x(rx-16'd1,!fx);wr_y(ry-16'd1,!fx);end
    if(ra-16'd1!=16'd0) begin ra<=ra-16'd1;rpc<=rpc-16'd3;end else ra<=16'hffff;
   end
   // ---------------------------------------------------------- divide: restoring 32/16, 32 steps
   S_DIV1: begin
    if(dcnt==6'd32) st<=S_DIV2;
    else begin
     if({rem[15:0],dvd[31]}>={1'b0,dvs}) begin rem<={rem[15:0],dvd[31]}-{1'b0,dvs};quo<={quo[30:0],1'b1};end
     else begin rem<={rem[15:0],dvd[31]};quo<={quo[30:0],1'b0};end
     dvd<={dvd[30:0],1'b0};dcnt<=dcnt+6'd1;
    end
   end
   S_DIV2: begin  // MAME: V = C = quotient (or remainder) does not fit; N cleared when no overflow
    if(w16 ? (quo[31:16]!=16'd0) : (quo[31:8]!=24'd0)) begin fv<=1;fc<=1;end
    else begin fv<=0;fc<=0;fn<=0;end
    if(w16) begin ra<=quo[15:0];rb<=rem[15:0];fz<=(quo[15:0]==16'd0);end
    else begin ra[7:0]<=quo[7:0];rb[7:0]<=rem[7:0];fz<=(quo[7:0]==8'd0);end
    st<=S_DONE;
   end
   // ---------------------------------------------------------- RLA: rotate A left n times, 6 cycles each
   S_RLA1: begin imm<=w16 ? mdata : {8'd0,mdata[7:0]};st<=S_RLA2;end
   S_RLA2: begin
    if(imm==16'd0) st<=S_DONE;
    else begin imm<=imm-16'd1;budget<=budget+20'd6;if(w16) ra<={ra[14:0],ra[15]}; else ra[7:0]<={ra[6:0],ra[7]};end
   end
   // ---------------------------------------------------------- interrupt entry (PG pushed by start_int)
   S_INT2: push8(rpc[15:8],S_INT3);
   S_INT3: push8(rpc[7:0],S_INT4);
   S_INT4: push8({5'd0,ipl},S_INT5);
   S_INT5: push8(ps,S_INT6);
   S_INT6: begin fi<=1;if(!sw_int) ipl<=irq_pri_l;rd(1'b1,{8'd0,vector},S_INT7);end
   S_INT7: begin rpg<=8'd0;rpc<=mdata;sw_int<=0;st<=S_DONE;end
   // ---------------------------------------------------------- memory sub-machine
   S_MEM1: begin
    if(!bus_req) begin
     bus_req<=1;bus_we<=m_we;
     if(m_w16 && !m_addr[0]) begin bus_addr<=m_addr;bus_be<=2'b11;bus_wdata<=m_wdata;end
     else if(!m_addr[0]) begin bus_addr<=m_addr;bus_be<=2'b01;bus_wdata<={8'd0,m_wdata[7:0]};end
     else begin bus_addr<={m_addr[23:1],1'b0};bus_be<=2'b10;bus_wdata<={m_wdata[7:0],8'd0};end
    end else if(bus_ack) begin
     bus_req<=0;
     if(m_w16 && !m_addr[0]) mdata<=bus_rdata;
     else if(!m_addr[0]) mdata<={8'd0,bus_rdata[7:0]};
     else mdata<={8'd0,bus_rdata[15:8]};
     if(m_w16 && m_addr[0]) st<=S_MEM2; else st<=ret;
    end
   end
   S_MEM2: begin  // second byte of an unaligned 16-bit access: next even address, low lane
    if(!bus_req) begin bus_req<=1;bus_we<=m_we;bus_addr<=m_addr+24'd1;bus_be<=2'b01;bus_wdata<={8'd0,m_wdata[15:8]};end
    else if(bus_ack) begin bus_req<=0;mdata[15:8]<=bus_rdata[7:0];st<=ret;end
   end
   // ---------------------------------------------------------- instruction boundary
   S_DONE: begin
    // One settle clock first: irq_take is registered in the SFR block, so it
    // reflects this instruction's ipl/I-flag updates from the second cycle on.
    if(!done_settled) done_settled<=1;
    else if(spent>=budget) begin
     if(spent>budget) overrun_count<=overrun_count+16'd1;
     done_settled<=0;
     instr_commit<=1;spent<=0;pfx<=0;step<=0;phase<=0;
     if(irq_take) begin irq_ack<=1;irq_ack_line<=irq_line;budget<=20'd13;start_int(vec_of(irq_line),1'b0,irq_pri);end
     else begin budget<=0;st<=S_FETCH;end
    end
   end
   S_WAI: begin   // MAME: the CPU sleeps until an interrupt is actually taken
    if(irq_take) begin
     instr_commit<=1;spent<=0;pfx<=0;budget<=20'd13;
     irq_ack<=1;irq_ack_line<=irq_line;start_int(vec_of(irq_line),1'b0,irq_pri);
    end
   end
   S_HALT: ;
   default: st<=S_DONE;
  endcase
 end
endmodule
