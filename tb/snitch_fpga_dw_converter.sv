// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors: Diyou Shen <dishe@iis.ee.ethz.ch>
// Description: the data width converter to re-assemble the 32b mem access
// This module will be deployed on the eFPGA side

module snitch_fpga_dw_converter
#(
  // Address Width from the ASIC side
  parameter int unsigned AsicAW = 8,
  // the data width from the ASIC side
  parameter int unsigned AsicDW = 4,
  // Address Width from the memory side
  parameter int unsigned MemAW  = 10,
  // the data width on the memory side
  parameter int unsigned MemDW  = 32,
  // are we using half handshaking? If so, response will not check ready signal
  parameter int unsigned HalfHS = 1,
  // Derived parameter, DO NOT CHANGE!
  parameter int unsigned Stages     = MemDW / AsicDW,
  // Derived parameter, DO NOT CHANGE!
  parameter int unsigned StrbWidth  = MemDW / 8

) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  // ASIC Side
  input  logic [   AsicAW-1:0]  asic_req_addr_i,
  input  logic [   AsicDW-1:0]  asic_req_data_i,
  input  logic                  asic_req_write_i,
  input  logic                  asic_req_wstrb_i,
  input  logic                  asic_req_valid_i,
  output logic                  asic_req_ready_o,
  output logic [   AsicDW-1:0]  asic_rsp_data_o,
  output logic                  asic_rsp_last_o,
  output logic                  asic_rsp_valid_o,
  input  logic                  asic_rsp_ready_i,

  // Mem Side
  output logic [    MemAW-1:0]  mem_req_addr_o,
  output logic [    MemDW-1:0]  mem_req_data_o,
  output logic                  mem_req_write_o,
  output logic [StrbWidth-1:0]  mem_req_wstrb_o,
  output logic                  mem_req_valid_o,
  input  logic                  mem_req_ready_i,
  input  logic [    MemDW-1:0]  mem_rsp_data_i,
  input  logic                  mem_rsp_valid_i,
  output logic                  mem_rsp_ready_o
);

  // Counter type used to count the data transfers
  typedef logic [$clog2(Stages)-1:0] cnt_t;

  // The state to combine the request (write)
  // Is it necessary? Maybe we can just adjust the width
  // and issue multiple writes to memory
  typedef enum logic {
    REQIDLE, COMBINE
  } fsm_req_e;

  fsm_req_e req_state_d, req_state_q;

  // Do not accept next request if we are busy
  logic busy_d, busy_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_req_fsm
    if(~rst_ni) begin
      req_state_q <= REQIDLE;
    end else begin
      req_state_q <= req_state_d;
    end
  end

  always_comb begin : req_fsm_comb

  end


  typedef enum logic {
    RSPIDLE, SEPARATE
  } fsm_rsp_e;

  fsm_rsp_e rsp_state_d, rsp_state_q;
  cnt_t     rsp_cnt_d,   rsp_cnt_q;

  // We do not need to store the highest AsicDW bits
  // They will be sent out once received
  logic [MemDW-AsicDW-1:0] rsp_data_d, rsp_data_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_rsp_fsm
    if(~rst_ni) begin
      rsp_state_q <= RSPIDLE;
      rsp_cnt_q   <= '0;
      rsp_data_q  <= '0;
      busy_q      <= '0;
    end else begin
      rsp_state_q <= rsp_state_d;
      rsp_cnt_q   <= rsp_cnt_d;
      rsp_data_q  <= rsp_data_d;
      busy_q      <= busy_d;
    end
  end



  always_comb begin : fsm_comb
    busy_d      = busy_q;

    // Request
    req_state_d = req_state_q;

    mem_req_addr_o    = '0;
    mem_req_data_o    = '0;
    mem_req_write_o   = '0;
    mem_req_wstrb_o   = '0;
    mem_req_valid_o   = '0;
    asic_req_ready_o  = '0;

    case (req_state_q)
      REQIDLE: begin
        mem_req_addr_o    = asic_req_addr_i;
        mem_req_data_o    = asic_req_data_i;
        mem_req_write_o   = asic_req_write_i;
        mem_req_wstrb_o   = {Stages{asic_req_wstrb_i}};
        // Do not accept if we are busy
        mem_req_valid_o   = busy_q ? '0   : asic_req_valid_i;
        asic_req_ready_o  = busy_q ? 1'b0 : mem_req_ready_i;

        if (mem_req_valid_o) begin
          // Start to handle one request
          busy_d = 1'b1;
        end
      end

      // not used for now
      COMBINE: begin

      end

    endcase


    // Response
    rsp_state_d = rsp_state_q;
    rsp_cnt_d   = rsp_cnt_q;
    rsp_data_d  = rsp_data_q;

    // default output values
    asic_rsp_data_o   = '0;
    asic_rsp_valid_o  = '0;
    asic_rsp_last_o   = 1'b0;
    /// do not ack the response yet 
    mem_rsp_ready_o   = '0;


    case (rsp_state_q)
      RSPIDLE: begin
        // if we receive a valid response
        // send out the first part and switch state
        if (mem_rsp_valid_i) begin
          // Send the highest AsicDW bits in this transaction
          asic_rsp_data_o   = mem_rsp_data_i[(MemDW-1)-:AsicDW];
          // Valid response
          asic_rsp_valid_o  = 1'b1;
          // Store the unprocessed data
          rsp_data_d        = mem_rsp_data_i[(MemDW-AsicDW-1):0];

          if (asic_rsp_ready_i) begin
            // Do not switch until a handshaking is established
            rsp_state_d     = SEPARATE;
            // we already sent out first part, substract one additional count.
            rsp_cnt_d       = (Stages-1);
            // We have stored all information, safe to ACK the response
            mem_rsp_ready_o = 1'b1;
          end
        end
      end

      SEPARATE: begin
        // Send the next transaction parts untill it finishes
        asic_rsp_data_o   = rsp_data_q[(MemDW-AsicDW-1)-:AsicDW];
        // Valid response
        asic_rsp_valid_o  = 1'b1;
        if (rsp_cnt_q == 1'b1) begin
          // last transaction for this request
          asic_rsp_last_o = 1'b1;
        end
        
        if (asic_rsp_ready_i) begin
          // Shift up the data for next round
          rsp_data_d      = rsp_data_q << AsicDW;
          // Substract the counter
          rsp_cnt_d       = rsp_cnt_q - 1;

          // Are we done with the transaction?
          if (rsp_cnt_d == '0) begin
            rsp_state_d   = RSPIDLE;
            rsp_cnt_d     = '0;
            rsp_data_d    = '0;
            busy_d        = 1'b0;
          end
        end
      end
    endcase
  end



endmodule
