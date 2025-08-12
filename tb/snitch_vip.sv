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

  always_comb begin : init_values
    mem_init    = '0;

    mem_init[3:0] = 256'h00000000_00000000_00000000_00000000_00050067_10500073_08050513_00000517;

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

        if (rsp_ready_i) begin
          // response is accepted
          rsp_data_d  = '0;
          rsp_valid_d = '0;
        end
      end
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
