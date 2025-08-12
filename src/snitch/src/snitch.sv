// Copyright 2020 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors: Florian Zaruba <zarubaf@iis.ee.ethz.ch>
//          Sergio Mazzola <smazzola@student.ethz.ch>
//          Marco Bertuletti <mbertuletti@iis.ee.ethz.ch>
// Description: Top-Level of Snitch Integer Core RV32E

// `include "./src/registers.svh"

// `SNITCH_ENABLE_PERF Enables mcycle, minstret performance counters (read only)
// `SNITCH_ENABLE_STALL_COUNTER Enables stall_ins, stall_raw, stall_lsu performance counters (read only)

// Flip-Flop with asynchronous active-high reset
// __q: Q output of FF
// __d: D input of FF
// __reset_value: value assigned upon reset
// __clk: clock input
// __arst: asynchronous reset, active-high
`define FFAR(__q, __d, __reset_value, __clk, __arst)     \
  always_ff @(posedge (__clk) or posedge (__arst)) begin \
    if (__arst) begin                                    \
      __q <= (__reset_value);                            \
    end else begin                                       \
      __q <= (__d);                                      \
    end                                                  \
  end

// DEPRECATED - use `FF instead
// Flip-Flop with asynchronous active-low reset
// __q: Q output of FF
// __d: D input of FF
// __reset_value: value assigned upon reset
// __clk: clock input
// __arst_n: asynchronous reset, active-low
`define FFARN(__q, __d, __reset_value, __clk, __arst_n) \
  `FF(__q, __d, __reset_value, __clk, __arst_n)


module snitch
#(
  parameter logic [31:0] BootAddr  = 32'h0000_1000,
  parameter logic [31:0] MTVEC     = BootAddr, // Exception Base Address (see privileged spec 3.1.7)
  parameter bit          RVE       = 0,   // Reduced-register Extension
  parameter bit          RVM       = 1,   // Enable IntegerMmultiplication & Division Extension

  parameter bit          RVV       = 0,   // Enable Vector Extension
  parameter bit          XFVEC     = 0,
  parameter bit          XFDOTP    = 0,
  parameter bit          XFAUX     = 0,
  /// Enable F Extension.
  parameter bit          RVF       = 0,
  /// Enable D Extension.
  parameter bit          RVD       = 0,
  parameter bit          XF16      = 0,
  parameter bit          XF16ALT   = 0,
  parameter bit          XF8       = 0,
  parameter bit          XF8ALT    = 0,
  /// Enable div/sqrt unit (buggy - use with caution)
  parameter bit          XDivSqrt  = 0,
  parameter int    RegNrWritePorts = 2,   // Implement one or two write ports into the register file
  //parameter type         acc_issue_rsp_t = logic,
  // Dependant parameters.
  localparam bit FP_EN             = 0  // Enable FP in general,
) (
  input  logic          clk_i,
  input  logic          rst_i,
  input  logic [31:0]   hart_id_i,
  // Instruction Refill Port
  output logic [31:0]   inst_addr_o,
  input  logic [31:0]   inst_data_i,
  output logic          inst_valid_o,
  input  logic          inst_ready_i,

  /// Accelerator Interface - Master Port
  /// Independent channels for transaction request and read completion.
  /// AXI-like handshaking.
  /// Same IDs need to be handled in-order.
  output logic [31:0]   acc_qaddr_o,
  output logic [4:0]    acc_qid_o,
  input  logic [4:0]    acc_pid_i,
  output logic [31:0]   acc_qdata_op_o,
  output logic [31:0]   acc_qdata_arga_o,
  output logic [31:0]   acc_qdata_argb_o,
  output logic [31:0]   acc_qdata_argc_o,
  output logic          acc_qvalid_o,
  input  logic          acc_qready_i,
  input  logic [31:0]   acc_pdata_i,
  input  logic          acc_pwrite_i,
  input  logic          acc_perror_i,
  input  logic          acc_pvalid_i,
  output logic          acc_pready_o,
  // input  logic          acc_qdata_rsp_i,
  // input  logic [1:0]    acc_mem_finished_i,
  // input  logic [1:0]    acc_mem_str_finished_i,
  /// TCDM Data Interface
  /// Write transactions do not return data on the `P Channel`
  /// Transactions need to be handled strictly in-order.
  output logic [31:0]   data_qaddr_o,
  output logic          data_qwrite_o,
  output logic [3:0]    data_qamo_o,
  output logic [31:0]   data_qdata_o,
  output logic [3:0]    data_qstrb_o,
  output logic [0:0]    data_qid_o,
  output logic          data_qvalid_o,
  input  logic          data_qready_i,
  input  logic [31:0]   data_pdata_i,
  input  logic          data_perror_i,
  input  logic  [0:0]   data_pid_i,
  input  logic          data_pvalid_i,
  output logic          data_pready_o,
  input  logic          wake_up_sync_i // synchronous wake-up interrupt
  // FPU **un-timed** Side-channel
  // output fpnew_pkg::roundmode_e    fpu_rnd_mode_o,
  // output fpnew_pkg::fmt_mode_t     fpu_fmt_mode_o,
  // input  fpnew_pkg::status_t       fpu_status_i,
  // Core event strobes
);

  localparam int RegWidth = RVE ? 4 : 5;
  localparam int RegNrReadPorts = snitch_pkg::XPULPIMG ? 3 : 2;
  localparam logic [RegWidth-1:0] SP = 2;
  localparam int OutstandingWfi = 8;

  logic illegal_inst;
  logic zero_lsb;

  // Instruction fetch
  logic [31:0] pc_d, pc_q;
  logic wfi_d, wfi_q;
  logic [$clog2(OutstandingWfi)-1:0] wake_up_d, wake_up_q;
  logic [31:0] consec_pc;
  // Immediates
  logic [31:0] iimm, uimm, jimm, bimm, simm, pbimm;
  /* verilator lint_off WIDTH */
  assign iimm = $signed({inst_data_i[31:20]});
  assign uimm = {inst_data_i[31:12], 12'b0};
  assign jimm = $signed({inst_data_i[31],
                                  inst_data_i[19:12], inst_data_i[20], inst_data_i[30:21], 1'b0});
  assign bimm = $signed({inst_data_i[31],
                                    inst_data_i[7], inst_data_i[30:25], inst_data_i[11:8], 1'b0});
  assign simm = $signed({inst_data_i[31:25], inst_data_i[11:7]});
  assign pbimm = $signed(inst_data_i[24:20]); // Xpulpimg immediate branching signed immediate
  /* verilator lint_on WIDTH */

  logic [31:0] opa, opb;
  logic [32:0] adder_result;
  logic [31:0] alu_result;

  logic [RegWidth-1:0] rd, rs1, rs2;
  logic stall, lsu_stall, fence_stall;
  // Register connections
  logic [RegNrReadPorts-1:0][RegWidth-1:0]  gpr_raddr;
  logic [RegNrReadPorts-1:0][31:0]          gpr_rdata;
  logic [RegNrWritePorts-1:0][RegWidth-1:0] gpr_waddr;
  logic [RegNrWritePorts-1:0][31:0]         gpr_wdata;
  logic [RegNrWritePorts-1:0]               gpr_we;
  logic [2**RegWidth-1:0]                   sb_d, sb_q;

  // Load/Store Defines
  logic is_load, is_store, is_signed, is_postincr;
  logic is_fp_load, is_fp_store;
  logic is_acc_mem, is_acc;
  logic ls_misaligned;
  logic ld_addr_misaligned;
  logic st_addr_misaligned;

  enum logic [1:0] {
    Byte = 2'b00,
    HalfWord = 2'b01,
    Word = 2'b10,
    Double = 2'b11
  } ls_size;

  enum logic [3:0] {
    AMONone = 4'h0,
    AMOSwap = 4'h1,
    AMOAdd  = 4'h2,
    AMOAnd  = 4'h3,
    AMOOr   = 4'h4,
    AMOXor  = 4'h5,
    AMOMax  = 4'h6,
    AMOMaxu = 4'h7,
    AMOMin  = 4'h8,
    AMOMinu = 4'h9,
    AMOLR   = 4'hA,
    AMOSC   = 4'hB
  } ls_amo;

  logic [31:0] ld_result;
  logic lsu_qready, lsu_qvalid;
  logic lsu_pvalid, lsu_pready;
  logic lsu_empty;
  logic [RegWidth-1:0] lsu_rd;
  logic [31:0] lsu_qaddr;


  logic retire_load; // retire a load instruction
  logic retire_p; // retire from post-increment instructions
  logic retire_i; // retire the rest of the base instruction set
  logic retire_acc; // retire an instruction we offloaded

  logic acc_stall;
  logic valid_instr;
  logic exception;

  // ALU Operations
  enum logic [3:0]  {
    // Arithmetical operations
    Add, Sub,
    // Shifts
    Sll, Srl, Sra,
    // Logical operations
    LXor, LOr, LAnd, LNAnd,
    // Comparisons
    Eq, Neq, Ge, Geu,
    Slt, Sltu,
    // Miscellaneous
    BypassA
  } alu_op;

  enum logic [3:0] {
    None, Reg, IImmediate, UImmediate, JImmediate, SImmediate, SFImmediate, PC, CSR, CSRImmediate, PBImmediate, RegRd, RegRs2
  } opa_select, opb_select, opc_select;

  logic write_rd; // write rd destination this cycle
  logic uses_rd;
  logic write_rs1; // write rs1 destination this cycle
  logic uses_rs1;
  enum logic [1:0] {Consec, Alu, Exception} next_pc;

  enum logic [1:0] {RdAlu, RdConsecPC, RdBypass} rd_select;
  logic [31:0] rd_bypass;

  logic is_branch;

  logic [31:0] csr_rvalue;
  logic csr_en;

  // Registers
  `FFAR(pc_q, pc_d, BootAddr, clk_i, rst_i)
  `FFAR(wfi_q, wfi_d, '0, clk_i, rst_i)
  `FFAR(wake_up_q, wake_up_d, '0, clk_i, rst_i)
  `FFAR(sb_q, sb_d, '0, clk_i, rst_i)

  // accelerator offloading interface
  // register int destination in scoreboard
  logic  acc_register_rd;
  // LSU stalling due to accelerator memory requests
  logic acc_mem_stall;
  // Offloaded mem operation is a store operation
  logic acc_mem_store;

  assign acc_qaddr_o = hart_id_i;
  assign acc_qid_o = rd;
  assign acc_qdata_op_o = inst_data_i;
  assign acc_qdata_arga_o = {{32{gpr_rdata[0][31]}}, gpr_rdata[0]};
  assign acc_qdata_argb_o = opb_select inside {IImmediate, SImmediate, SFImmediate} ?
    {{32{alu_result[31]}}, alu_result} : {{32{gpr_rdata[1][31]}}, gpr_rdata[1]};

  assign acc_qdata_argc_o = '0;


  // instruction fetch interface
  assign inst_addr_o = pc_q;
  assign inst_valid_o = ~wfi_q;

  // --------------------
  // Control
  // --------------------
  // Scoreboard: Keep track of rd dependencies (only loads at the moment)
  logic operands_ready;
  logic dst_ready;
  logic opa_ready, opb_ready, opc_ready;
  logic dstrd_ready, dstrs1_ready;

  always_comb begin
    sb_d = sb_q;
    if (retire_load) sb_d[lsu_rd] = 1'b0;
    // only place the reservation if we actually executed the load or offload instruction
    if ((is_load | acc_register_rd) && !stall && !exception) sb_d[rd] = 1'b1;
    if (retire_acc) sb_d[acc_pid_i[RegWidth-1:0]] = 1'b0;
    sb_d[0] = 1'b0;
  end
  // TODO(zarubaf): This can probably be described a bit more efficient
  assign opa_ready = (opa_select != Reg) | ~sb_q[rs1];
  assign opb_ready = ((opb_select != Reg & opb_select != SImmediate) | ~sb_q[rs2]) & ((opb_select != RegRd) | ~sb_q[rd]);
  assign opc_ready = ((opc_select != Reg) | ~sb_q[rd]) & ((opc_select != RegRs2) | ~sb_q[rs2]);
  assign operands_ready = opa_ready & opb_ready & opc_ready;
  // either we are not using the destination register or we need to make
  // sure that its destination operand is not marked busy in the scoreboard.
  assign dstrd_ready = ~uses_rd | (uses_rd & ~sb_q[rd]);
  assign dstrs1_ready = ~uses_rs1 | (uses_rs1 & ~sb_q[rs1]);
  assign dst_ready = dstrd_ready & dstrs1_ready;

  assign valid_instr = (inst_ready_i & inst_valid_o) & operands_ready & dst_ready;

  assign acc_stall = '0;

  // Stall the stage if we either didn't get a valid instruction or the LSU/Accelerator is not ready
  always_comb begin
    lsu_stall = (lsu_qvalid & ~lsu_qready);
    stall = ~valid_instr | lsu_stall | acc_stall | fence_stall;
    // if (acc_mem_stall) begin
    //   // If acc memory is stalling, it is not necessary to stall the snich
    //   // only memory instructions cannot be forward to avoid data hazards
    //   if (is_acc_mem | is_store | is_load) begin
    //     // If is a acc insn or snitch mem insn, we stall
    //     lsu_stall = 1'b1;
    //     stall     = 1'b1;
    //   end
    // end
  end

  // --------------------
  // Instruction Frontend
  // --------------------
  assign consec_pc = pc_q + ((is_branch & alu_result[0]) ? bimm : 'd4);

  always_comb begin
    pc_d = pc_q;
    // if we got a valid instruction word increment the PC unless we are waiting for an event
    if (!stall && !wfi_q) begin
      casez (next_pc)
        Consec: pc_d = consec_pc;
        Alu: pc_d = alu_result & {{31{1'b1}}, ~zero_lsb};
        Exception: pc_d = MTVEC;
      endcase
    end
  end

  // --------------------
  // Decoder
  // --------------------
  assign rd = inst_data_i[7 + RegWidth - 1:7];
  assign rs1 = inst_data_i[15 + RegWidth - 1:15];
  assign rs2 = inst_data_i[20 + RegWidth - 1:20];

  always_comb begin
    illegal_inst = 1'b0;
    alu_op = Add;
    opa_select = None;
    opb_select = None;
    opc_select = None;

    next_pc = Consec;

    // set up rd destination
    rd_select = RdAlu;
    write_rd = 1'b1;
    // if we are writing the field this cycle we need an int destination register
    uses_rd = write_rd;
    // set up rs1 destination
    write_rs1 = 1'b0;
    uses_rs1 = write_rs1;

    rd_bypass = '0;
    zero_lsb = 1'b0;
    is_branch = 1'b0;
    // LSU interface
    is_load = 1'b0;
    is_store = 1'b0;
    is_postincr = 1'b0;
    is_fp_load = 1'b0;
    is_fp_store = 1'b0;
    is_signed = 1'b0;
    ls_size = Byte;
    ls_amo = AMONone;

    is_acc_mem = 1'b0;
    is_acc     = 1'b0;

    acc_qvalid_o = 1'b0;
    acc_register_rd = 1'b0;
    acc_mem_store = 1'b0;

    csr_en = 1'b0;
    fence_stall = 1'b0;
    // Wake up if a wake-up is incoming or pending
    wfi_d = ((|wake_up_q) | wake_up_sync_i) ? 1'b0 : wfi_q;
    // Only store a pending wake-up if we are not asleep
    wake_up_d = (wake_up_sync_i && !wfi_q) ? wake_up_q + 1 : wake_up_q;

    unique casez (inst_data_i)
      riscv_instr::ADD: begin
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::ADDI: begin
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SUB: begin
        alu_op = Sub;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::XOR: begin
        opa_select = Reg;
        opb_select = Reg;
        alu_op = LXor;
      end
      riscv_instr::XORI: begin
        alu_op = LXor;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::OR: begin
        opa_select = Reg;
        opb_select = Reg;
        alu_op = LOr;
      end
      riscv_instr::ORI: begin
        alu_op = LOr;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::AND: begin
        alu_op = LAnd;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::ANDI: begin
        alu_op = LAnd;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SLT: begin
        alu_op = Slt;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SLTI: begin
        alu_op = Slt;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SLTU: begin
        alu_op = Sltu;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SLTIU: begin
        alu_op = Sltu;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SLL: begin
        alu_op = Sll;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SRL: begin
        alu_op = Srl;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SRA: begin
        alu_op = Sra;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SLLI: begin
        alu_op = Sll;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SRLI: begin
        alu_op = Srl;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::SRAI: begin
        alu_op = Sra;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::LUI: begin
        opa_select = None;
        opb_select = None;
        rd_select = RdBypass;
        rd_bypass = uimm;
      end
      riscv_instr::AUIPC: begin
        opa_select = UImmediate;
        opb_select = PC;
      end
      riscv_instr::JAL: begin
        rd_select = RdConsecPC;
        opa_select = JImmediate;
        opb_select = PC;
        next_pc = Alu;
      end
      riscv_instr::JALR: begin
        rd_select = RdConsecPC;
        opa_select = Reg;
        opb_select = IImmediate;
        next_pc = Alu;
        zero_lsb = 1'b1;
      end
      // use the ALU for comparisons
      riscv_instr::BEQ: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Eq;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::BNE: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Neq;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::BLT: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Slt;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::BLTU: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Sltu;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::BGE: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Ge;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::BGEU: begin
        is_branch = 1'b1;
        write_rd = 1'b0;
        alu_op = Geu;
        opa_select = Reg;
        opb_select = Reg;
      end
      // Load/Stores
      riscv_instr::SB: begin
        write_rd = 1'b0;
        is_store = 1'b1;
        opa_select = Reg;
        opb_select = SImmediate;
      end
      riscv_instr::SH: begin
        write_rd = 1'b0;
        is_store = 1'b1;
        ls_size = HalfWord;
        opa_select = Reg;
        opb_select = SImmediate;
      end
      riscv_instr::SW: begin
        write_rd = 1'b0;
        is_store = 1'b1;
        ls_size = Word;
        opa_select = Reg;
        opb_select = SImmediate;
      end
      riscv_instr::LB: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::LH: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = HalfWord;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::LW: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::LBU: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      riscv_instr::LHU: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        ls_size = HalfWord;
        opa_select = Reg;
        opb_select = IImmediate;
      end
      // CSR Instructions
      riscv_instr::CSRRW: begin // Atomic Read/Write CSR
        unique case (inst_data_i[31:20])
          default: begin
            opa_select = Reg;
            opb_select = None;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::CSRRWI: begin
        unique case (inst_data_i[31:20])
          default: begin
            opa_select = CSRImmediate;
            opb_select = None;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::CSRRS: begin  // Atomic Read and Set Bits in CSR
        unique case (inst_data_i[31:20])
          default: begin
            alu_op = LOr;
            opa_select = Reg;
            opb_select = CSR;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::CSRRSI: begin
        unique case (inst_data_i[31:20])
          snitch_pkg::CSR_SSR: begin
            write_rd = 1'b0;
            acc_qvalid_o = valid_instr;
          end
          default: begin
            alu_op = LOr;
            opa_select = CSRImmediate;
            opb_select = CSR;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::CSRRC: begin // Atomic Read and Clear Bits in CSR
        unique case (inst_data_i[31:20])
          default: begin
            alu_op = LNAnd;
            opa_select = Reg;
            opb_select = CSR;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::CSRRCI: begin
        unique case (inst_data_i[31:20])
          snitch_pkg::CSR_SSR: begin
            write_rd = 1'b0;
            acc_qvalid_o = valid_instr;
          end
          default: begin
            alu_op = LNAnd;
            opa_select = CSRImmediate;
            opb_select = CSR;
            rd_select = RdBypass;
            rd_bypass = csr_rvalue;
            csr_en = 1'b1;
          end
        endcase
      end
      riscv_instr::ECALL,
      riscv_instr::EBREAK: begin
        // TODO(zarubaf): Trap to precise address
        write_rd = 1'b0;
      end
      // NOP Instructions
      riscv_instr::FENCE: begin
        write_rd = 1'b0;
        // Stall until the LSU is empty
        fence_stall = !lsu_empty;
      end
      riscv_instr::WFI: begin
        if (valid_instr) begin
          wfi_d = 1'b1;
          if ((|wake_up_q)| wake_up_sync_i) begin
            // Do not sleep if a wake-up is pending
            wfi_d = 1'b0;
            if (|wake_up_q) begin
              // Decrement outstanding wake_up pulses
              wake_up_d = wake_up_q - 1;
            end
            if (wake_up_sync_i) begin
              // Keep counter constant due to simultaneous pulse
              wake_up_d = wake_up_q;
            end
          end
        end
      end
      // Atomics
      riscv_instr::AMOADD_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOAdd;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOXOR_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOXor;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOOR_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOOr;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOAND_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOAnd;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOMIN_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOMin;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOMAX_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOMax;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOMINU_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOMinu;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOMAXU_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOMaxu;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::AMOSWAP_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOSwap;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::LR_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOLR;
        opa_select = Reg;
        opb_select = Reg;
      end
      riscv_instr::SC_W: begin
        alu_op = BypassA;
        write_rd = 1'b0;
        uses_rd = 1'b1;
        is_load = 1'b1;
        is_signed = 1'b1;
        ls_size = Word;
        ls_amo = AMOSC;
        opa_select = Reg;
        opb_select = Reg;
      end
      // Off-load to IPU coprocessor
      riscv_instr::MUL,
      riscv_instr::MULH,
      riscv_instr::MULHSU,
      riscv_instr::MULHU,
      riscv_instr::DIV,
      riscv_instr::DIVU,
      riscv_instr::REM,
      riscv_instr::REMU,
      riscv_instr::MULW,
      riscv_instr::DIVW,
      riscv_instr::DIVUW,
      riscv_instr::REMW,
      riscv_instr::REMUW: begin
        write_rd = 1'b0;
        uses_rd = 1'b1;
        acc_qvalid_o = valid_instr;
        opa_select = Reg;
        opb_select = Reg;
        acc_register_rd = 1'b1;
      end


      // TODO(zarubaf): Illegal Instructions
      default: begin
        illegal_inst = 1'b1;
      end
    endcase

    // Sanitize illegal instructions so that they don't exert any side-effects.
    if (exception) begin
     write_rd = 1'b0;
     uses_rd = 1'b0;
     write_rs1 = 1'b0;
     uses_rs1 = 1'b0;
     acc_qvalid_o = 1'b0;
     next_pc = Exception;
    end
  end

  assign exception = illegal_inst | ld_addr_misaligned | st_addr_misaligned;

  // CSR logic
  logic csr_dump;
  logic csr_trace_en, csr_stack_limit_en;
  logic [31:0] csr_trace_q, csr_stack_limit_q;
  logic [11:0] csr_source_dest;

  assign csr_source_dest = inst_data_i[31:20];

  always_comb begin
    csr_rvalue = '0;
    csr_dump = 1'b0;
    csr_trace_en = 1'b0;
    csr_stack_limit_en = 1'b0;

    // TODO(zarubaf): Needs some more input handling, like illegal instruction exceptions.
    // Right now we skip this due to simplicity.
    if (csr_en) begin
      unique case (csr_source_dest)
        riscv_instr::CSR_MHARTID: begin
          csr_rvalue = hart_id_i;
        end
        riscv_instr::CSR_TRACE: begin
          csr_rvalue = csr_trace_q;
          csr_trace_en = 1'b1;
        end
        riscv_instr::CSR_STACKLIMIT: begin
          csr_rvalue = csr_stack_limit_q;
          csr_stack_limit_en = 1'b1;
        end
        default: begin
          csr_rvalue = '0;
          csr_dump = 1'b1;
        end
      endcase
    end
  end

  // CSR registers
  assign csr_trace_q = '0;
  assign csr_stack_limit_q = '0;


  // pragma translate_off
  always_ff @(posedge clk_i or posedge rst_i) begin
    // Display CSR write if the CSR does not exist
    if (!rst_i && csr_dump && inst_valid_o && inst_ready_i && !stall) begin
      $timeformat(-9, 0, " ns", 0);
      $display("[DUMP] %t Core %3d: 0x%3h = 0x%08h, %d", $time, hart_id_i, inst_data_i[31:20], alu_result, alu_result);
    end
  end
  // pragma translate_on

  snitch_regfile #(
    .DATA_WIDTH     ( 32              ),
    .NR_READ_PORTS  ( RegNrReadPorts  ),
    .NR_WRITE_PORTS ( RegNrWritePorts ),
    .ZERO_REG_ZERO  ( 1               ),
    .ADDR_WIDTH     ( RegWidth        )
  ) i_snitch_regfile (
    .clk_i,
    .rst_ni (!rst_i),
    .raddr_i   ( gpr_raddr ),
    .rdata_o   ( gpr_rdata ),
    .waddr_i   ( gpr_waddr ),
    .wdata_i   ( gpr_wdata ),
    .we_i      ( gpr_we    )
  );

  // --------------------
  // Operand Select
  // --------------------
  always_comb begin
    unique case (opa_select)
      None: opa = '0;
      Reg: opa = gpr_rdata[0];
      UImmediate: opa = uimm;
      JImmediate: opa = jimm;
      CSRImmediate: opa = {{{32-RegWidth}{1'b0}}, rs1};
      default: opa = '0;
    endcase
  end

  always_comb begin
    unique case (opb_select)
      None: opb = '0;
      Reg: opb = gpr_rdata[1];
      IImmediate: opb = iimm;
      SFImmediate, SImmediate: opb = simm;
      PC: opb = pc_q;
      CSR: opb = csr_rvalue;
      PBImmediate: opb = pbimm;
      default: opb = '0;
    endcase
  end

  assign gpr_raddr[0] = rs1;
  assign gpr_raddr[1] = rs2;
  // connect third read port only if present
  if (RegNrReadPorts >= 3) begin : gpr_raddr_2
    assign gpr_raddr[2] = rd;
  end

  // --------------------
  // ALU
  // --------------------
  // Main Shifter
  logic [31:0] shift_opa, shift_opa_reversed;
  logic [31:0] shift_right_result, shift_left_result;
  logic [32:0] shift_opa_ext, shift_right_result_ext;
  logic shift_left, shift_arithmetic; // shift control
  for (genvar i = 0; i < 32; i++) begin : gen_reverse_opa
    assign shift_opa_reversed[i] = opa[31-i];
    assign shift_left_result[i] = shift_right_result[31-i];
  end
  assign shift_opa = shift_left ? shift_opa_reversed : opa;
  assign shift_opa_ext = {shift_opa[31] & shift_arithmetic, shift_opa};
  assign shift_right_result_ext = $unsigned($signed(shift_opa_ext) >>> opb[4:0]);
  assign shift_right_result = shift_right_result_ext[31:0];

  // Main Adder
  logic [32:0] alu_opa, alu_opb;
  assign adder_result = alu_opa + alu_opb;

  // ALU
  /* verilator lint_off WIDTH */
  always_comb begin
    alu_opa = $signed(opa);
    alu_opb = $signed(opb);

    alu_result = adder_result[31:0];
    shift_left = 1'b0;
    shift_arithmetic = 1'b0;

    unique case (alu_op)
      // Arithmetical operations
      Sub: alu_opb = -$signed(opb);
      // Comparisons
      Slt: begin
        alu_opb = -$signed(opb);
        alu_result = {30'b0, adder_result[32]};
      end
      Ge: begin
        alu_opb = -$signed(opb);
        alu_result = {30'b0, ~adder_result[32]};
      end
      Sltu: begin
        alu_opa = $unsigned(opa);
        alu_opb = -$unsigned(opb);
        alu_result = {30'b0, adder_result[32]};
      end
      Geu: begin
        alu_opa = $unsigned(opa);
        alu_opb = -$unsigned(opb);
        alu_result = {30'b0, ~adder_result[32]};
      end
      // Shifts
      Sll: begin
        shift_left = 1'b1;
        alu_result = shift_left_result;
      end
      Srl: alu_result = shift_right_result;
      Sra: begin
        shift_arithmetic = 1'b1;
        alu_result = shift_right_result;
      end
      // Logical operations
      LXor: alu_result = opa ^ opb;
      LAnd: alu_result = opa & opb;
      LNAnd: alu_result = (~opa) & opb;
      LOr: alu_result = opa | opb;
      // Equal, not equal
      Eq: begin
        alu_opb = -$signed(opb);
        alu_result = ~|adder_result;
      end
      Neq: begin
        alu_opb = -$signed(opb);
        alu_result = |adder_result;
      end
      // Miscellaneous
      BypassA: begin
        alu_result = opa;
      end
      default: alu_result = adder_result[31:0];
    endcase
  end
  /* verilator lint_on WIDTH */

  // --------------------
  // LSU
  // --------------------
  logic [RegWidth-1:0] lsu_qtag;
  logic [31:0] lsu_qdata;

  // Send the return register if it is a read or an AMO
  assign lsu_qtag = (lsu_qvalid && (!is_store || ls_amo != AMONone)) ? rd : '0;
  // Send the data if it is a write or an AMO
  assign lsu_qdata = (lsu_qvalid && (is_store || ls_amo != AMONone)) ? gpr_rdata[1] : '0;
  localparam int unsigned NumIOLoadBit = 1;

  snitch_lsu #(
    .tag_t               ( logic[RegWidth-1:0]                ),
    .NumOutstandingLoads ( snitch_pkg::NumIntOutstandingLoads )
  ) i_snitch_lsu (
    .clk_i                                ,
    .rst_ni       ( !rst_i                ),
    .lsu_qtag_i   ( rd                    ),
    .lsu_qwrite_i ( is_store              ),
    .lsu_qsigned_i( is_signed             ),
    .lsu_qaddr_i  ( lsu_qaddr             ),
    .lsu_qdata_i  ( lsu_qdata             ),
    .lsu_qsize_i  ( ls_size               ),
    .lsu_qamo_i   ( ls_amo                ),
    .lsu_qvalid_i ( lsu_qvalid            ),
    .lsu_qready_o ( lsu_qready            ),
    .lsu_pdata_o  ( ld_result             ),
    .lsu_ptag_o   ( lsu_rd                ),
    .lsu_perror_o (                       ), // ignored for the moment
    .lsu_pvalid_o ( lsu_pvalid            ),
    .lsu_pready_i ( lsu_pready            ),
    .lsu_empty_o  ( lsu_empty             ),
    .data_qaddr_o                          ,
    .data_qwrite_o                         ,
    .data_qdata_o                          ,
    .data_qamo_o                           ,
    .data_qstrb_o                          ,
    .data_qid_o   ( data_qid_o[NumIOLoadBit-1:0])                         ,
    .data_qvalid_o                         ,
    .data_qready_i                         ,
    .data_pdata_i                          ,
    .data_perror_i                         ,
    // We may have more ID fields due to enlarged Spatz ROB
    .data_pid_i   ( data_pid_i[NumIOLoadBit-1:0]),
    .data_pvalid_i                         ,
    .data_pready_o
  );
  if (snitch_pkg::NumIntOutstandingLoads < snitch_pkg::RobDepth) begin
    assign data_qid_o[$clog2(snitch_pkg::RobDepth)-1:NumIOLoadBit] = '0;
  end

  // address can be alu_result (i.e. rs1 + iimm/simm) or rs1 (for post-increment load/stores)
  assign lsu_qaddr = lsu_qvalid ? (is_postincr ? gpr_rdata[0] : alu_result) : '0;
  assign lsu_qvalid = valid_instr & (is_load | is_store) & ~(ld_addr_misaligned | st_addr_misaligned);

  // NOTE(smazzola): write-backs "on rd from non-load or non-acc instructions" and "on rs1 from
  // post-increment instructions" in the same cycle should be mutually exclusive (currently valid
  // assumption since write-back to rs1 happens on the cycle in which the post-increment load/store
  // is issued, if that cycle is not a stall, and it is not postponed like offloaded instructions,
  // so no other instructions writing back on rd can be issued in the same cycle)
  // retire post-incremented address on rs1 if valid postincr instruction and LSU not stalling
  assign retire_p = write_rs1 & ~stall & (rs1 != 0);
  // we can retire if we are not stalling and if the instruction is writing a register
  assign retire_i = write_rd & valid_instr & (rd != 0);

  // -----------------------
  // Unaligned Address Check
  // -----------------------
  always_comb begin
    ls_misaligned = 1'b0;
    unique case (ls_size)
      HalfWord: if (alu_result[0] != 1'b0) ls_misaligned = 1'b1;
      Word: if (alu_result[1:0] != 2'b00) ls_misaligned = 1'b1;
      Double: if (alu_result[2:0] != 3'b000) ls_misaligned = 1'b1;
      default: ls_misaligned = 1'b0;
    endcase
  end

  assign st_addr_misaligned = ls_misaligned & (is_store | is_fp_store);
  assign ld_addr_misaligned = ls_misaligned & (is_load | is_fp_load);

  // pragma translate_off
  always_ff @(posedge clk_i or posedge rst_i) begin
    if (!rst_i && (ld_addr_misaligned || st_addr_misaligned) && valid_instr && inst_ready_i) begin
      $display("%t: [Misaligned Load/Store Core %0d] PC: %h Address: %h Data: %h", $time, hart_id_i, inst_addr_o, alu_result, inst_data_i);
    end
  end
  // pragma translate_on

  // --------------------
  // Write-Back
  // --------------------
  // Write-back data, can come from:
  // 1. ALU/Jump Target/Bypass
  // 2. LSU
  // 3. Accelerator Bus
  logic [31:0] alu_writeback;
  always_comb begin
    casez (rd_select)
      RdAlu: alu_writeback = alu_result;
      RdConsecPC: alu_writeback = consec_pc;
      RdBypass: alu_writeback = rd_bypass;
      default: alu_writeback = alu_result;
    endcase
  end

  if (RegNrWritePorts == 1) begin
    always_comb begin
      gpr_we[0] = 1'b0;
      // NOTE(smazzola): this works because write-backs on rd and rs1 in the same cycle are mutually
      // exclusive; if this should change, the following statement has to be written in another form
      gpr_waddr[0] = retire_p ? rs1 : rd; // choose whether to writeback at RF[rs1] for post-increment load/stores
      gpr_wdata[0] = alu_writeback;
      // external interfaces
      lsu_pready = 1'b0;
      acc_pready_o = 1'b0;
      retire_acc = 1'b0;
      retire_load = 1'b0;

      if (retire_i | retire_p) begin
        gpr_we[0] = 1'b1;
      end else begin
        // if we are not retiring another instruction retire the load now
        lsu_pready = 1'b1;
        if (lsu_pvalid) begin
          retire_load = 1'b1;
          gpr_we[0] = 1'b1;
          gpr_waddr[0] = lsu_rd;
          gpr_wdata[0] = ld_result[31:0];
        end else if (acc_pvalid_i & acc_pwrite_i) begin
          // if we are not retiring another instruction retire the accelerated one now
          retire_acc = 1'b1;
          gpr_we[0] = 1'b1;
          gpr_waddr[0] = acc_pid_i;
          gpr_wdata[0] = acc_pdata_i[31:0];
          acc_pready_o = 1'b1;
        end
      end
    end
  end else if (RegNrWritePorts == 2) begin
    always_comb begin
      gpr_we[0] = 1'b0;
      // NOTE(smazzola): this works because write-backs on rd and rs1 in the same cycle are mutually
      // exclusive; if this should change, the following statement has to be written in another form
      gpr_waddr[0] = retire_p ? rs1 : rd; // choose whether to writeback at RF[rs1] for post-increment load/stores
      gpr_wdata[0] = alu_writeback;
      gpr_we[1] = 1'b0;
      gpr_waddr[1] = lsu_rd;
      gpr_wdata[1] = ld_result[31:0];
      // external interfaces
      // Snitch and LSU have priority if Spatz is not used
      lsu_pready = 1'b1;
      acc_pready_o = 1'b0;
      retire_acc = 1'b0;
      retire_load = 1'b0;

      if (retire_i | retire_p) begin
        gpr_we[0] = 1'b1;
        if (lsu_pvalid) begin
          retire_load = 1'b1;
          gpr_we[1] = 1'b1;
          lsu_pready = 1'b1;
        end else if (acc_pvalid_i & acc_pwrite_i) begin
          retire_acc = 1'b1;
          gpr_we[1] = 1'b1;
          gpr_waddr[1] = acc_pid_i;
          gpr_wdata[1] = acc_pdata_i[31:0];
          acc_pready_o = 1'b1;
        end
      // if we are not retiring another instruction retire the load now
      end else begin
        if (acc_pvalid_i & acc_pwrite_i) begin
          retire_acc = 1'b1;
          gpr_we[0] = 1'b1;
          gpr_waddr[0] = acc_pid_i;
          gpr_wdata[0] = acc_pdata_i[31:0];
          acc_pready_o = 1'b1;
        end
        if (lsu_pvalid) begin
          retire_load = 1'b1;
          gpr_we[1] = 1'b1;
          lsu_pready = 1'b1;
        end
      end
    end
  end else begin
    $fatal(1, "[snitch] Unsupported RegNrWritePorts.");
  end

  // --------------------
  // Stack overflow check
  // --------------------

  // pragma translate_off
  for (genvar i = 0; i < RegNrWritePorts; i++) begin : gen_stack_overflow_check
    logic [31:0] sp_new_value;
    assign sp_new_value = gpr_wdata[i];
    always_ff @(posedge clk_i or posedge rst_i) begin
      if (!rst_i && gpr_we[i] && gpr_waddr[i] == SP && csr_stack_limit_q != 32'hFFFF_FFFF && ($signed(sp_new_value) < $signed(csr_stack_limit_q))) begin
        $warning("[Stackoverflow: Core %0d] Set SP to 0x%08h, limit is 0x%08h", hart_id_i, sp_new_value, csr_stack_limit_q);
      end
    end
  end
  // pragma translate_on

endmodule
