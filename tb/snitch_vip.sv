// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors: Diyou Shen <dishe@iis.ee.ethz.ch>
// Description: Simple virification IP for tiny tapeout of Snitch

module snitch_vip
#(

) (
  input  logic          clk_i,
  input  logic          rst_ni,
  input  logic [ 9:0]   req_addr_i,
  input  logic [31:0]   req_data_i,
  input  logic          req_write_i,
  input  logic          req_wstrb_i,
  input  logic          req_valid_i,
  output logic          req_ready_o,
  output logic [31:0]   rsp_data_o,
  output logic          rsp_valid_o,
  input  logic          rsp_ready_i
);

  typedef logic [31:0] data_t;

  parameter int MemDepth = 2**10;
  data_t [MemDepth-1:0] mem_d, mem_q, mem_init;

  /* Program
  00000000 <_start>:
     0: 00000517            auipc a0,0x0
     4: 04c52503            lw  a0,76(a0) # 4c <_GLOBAL_OFFSET_TABLE_+0x4>
     8: 00050067            jr  a0

  0000000c <__kernel>:
     c: 10000593            li  a1,256
    10: 0005a283            lw  t0,0(a1)
    14: 0045a303            lw  t1,4(a1)
    18: 006283b3            add t2,t0,t1
    1c: 0075a423            sw  t2,8(a1)

  Disassembly of section .rodata:

  00000020 <BOOTDATA>:
    20: 1000                  .2byte  0x1000
    22: 0000                  .2byte  0x0
    24: 0004                  .2byte  0x4
    26: 0000                  .2byte  0x0
    28: 0010                  .2byte  0x10
    2a: 0000                  .2byte  0x0
    2c: 0000                  .2byte  0x0
    2e: 5100                  .2byte  0x5100
    30: 0000                  .2byte  0x0
    32: 0002                  .2byte  0x2
    34: 0000                  .2byte  0x0
    36: 0000                  .2byte  0x0
    38: 0000                  .2byte  0x0
    3a: 8000                  .2byte  0x8000
    ...
    44: 0001                  .2byte  0x1
    ...

  Disassembly of section .got:

  00000048 <_GLOBAL_OFFSET_TABLE_>:
    48: 0000                  .2byte  0x0
    4a: 0000                  .2byte  0x0
    4c: 000c                  .2byte  0xc
  */


  always_comb begin : init_values
    mem_init    = '0;

    // Code
    mem_init[3 : 0] = 128'h10000593_00050067_04c52503_00000517;
    mem_init[7 : 4] = 128'h0075a423_006283b3_0045a303_0005a283;

    // Boot location
    mem_init[19:16] = 128'h0000000C_0000000C_0000000C_0000000C;

    // Data
    mem_init[68:64] = 128'h00000004_00000003_00000002_00000001;

  end

  assign req_ready_o = 1'b1;

  // always_comb begin : data_write

  //   if (req_valid_i & req_write_i) begin
  //     // Valid write request
  //     mem_d[req_addr_i] = req_data_i;
  //   end
  // end

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_mem
    if(~rst_ni) begin
      mem_q <= mem_init;
    end else begin
      mem_q <= mem_d;
    end
  end

  // Pipeline the response for one cycle
  data_t rsp_data_d,  rsp_data_q;
  logic  rsp_valid_d, rsp_valid_q;

  always_comb begin: data_read
    mem_d = mem_q;

    rsp_data_d  = rsp_data_q;
    rsp_valid_d = rsp_valid_q;

    if (req_valid_i) begin
      if (req_write_i) begin
        // Valid write request
        mem_d[req_addr_i] = req_data_i;
        // rsp_data_d
      end else begin
        // Valid read request
        rsp_data_d  = mem_q[req_addr_i];
        rsp_valid_d = 1'b1;

      end
    end

    if (rsp_ready_i) begin
      // response is accepted
      rsp_data_d  = '0;
      rsp_valid_d = '0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_read
    if(~rst_ni) begin
      rsp_data_q  <= '0;
      rsp_valid_q <= '0;
    end else begin
      rsp_data_q  <= rsp_data_d;
      rsp_valid_q <= rsp_valid_d;
    end
  end

  assign rsp_data_o  = rsp_data_q;
  assign rsp_valid_o = rsp_valid_q;

endmodule
