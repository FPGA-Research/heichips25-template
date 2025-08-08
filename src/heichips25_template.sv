// SPDX-FileCopyrightText: © 2025 XXX Authors
// SPDX-License-Identifier: Apache-2.0

// Adapted from the Tiny Tapeout template

`default_nettype none

module heichips25_template (
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered, so you can ignore it
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  // Instruction interface (should be muxed with data)
  logic [31:0] inst_addr, inst_data;
  logic inst_valid, inst_ready;

  // Data interface (q means request, p means response)
  logic [31:0] data_qaddr, data_qdata, data_pdata;
  logic [3 :0] data_strb;
  logic data_write;
  logic data_qvalid, data_qready, data_pvalid, data_pready;

  logic wake_up_sync;

  localparam int unsigned BootAddr = 32'h0000_0000;

  snitch #(
    .BootAddr ( BootAddr ),
    .MTVEC    ( BootAddr ),
    .RVE      ( 1'b0     ),
    .RVM      ( 1'b1     )
  ) i_snitch (
    .clk_i            ( clk           ),
    .rst_i            ( !rst_n        ),
    .hart_id_i        ( '0            ),
    .inst_addr_o      ( inst_addr     ),
    .inst_data_i      ( inst_data     ),
    .inst_valid_o     ( inst_valid    ),
    .inst_ready_i     ( inst_ready    ),
    .acc_qaddr_o      (               ),
    .acc_qid_o        (               ),
    .acc_qdata_op_o   (               ),
    .acc_qdata_arga_o (               ),
    .acc_qdata_argb_o (               ),
    .acc_qdata_argc_o (               ),
    .acc_qvalid_o     (               ),
    .acc_qready_i     ( '0            ),
    .acc_pdata_i      ( '0            ),
    .acc_pid_i        ( '0            ),
    .acc_perror_i     ( '0            ),
    .acc_pvalid_i     ( '0            ),
    .acc_pready_o     (               ),
    .data_qaddr_o     ( data_qaddr    ),
    .data_qwrite_o    ( data_write    ),
    .data_qamo_o      (               ),
    .data_qdata_o     ( data_qdata    ),
    .data_qstrb_o     ( data_strb     ),
    .data_qid_o       (               ),
    .data_qvalid_o    ( data_qvalid   ),
    .data_qready_i    ( data_qready   ),
    .data_pdata_i     ( data_pdata    ),
    .data_perror_i    ( '0            ),
    .data_pid_i       ( '0            ),
    .data_pvalid_i    ( data_pvalid   ),
    .data_pready_o    ( data_pready   ),
    .wake_up_sync_i   ( wake_up_sync  )
  );

  // === TODO3: FIFO to serialize 32-bit to 4-bit ===
  

  typedef enum logic {
    IDLE, SEND
  } fsm_state_e;

  fsm_state_e state, next_state;
  logic [31:0] shift_reg_q, shift_reg_d;
  // write strb
  logic [7:0]  strb_reg_q, strb_reg_d, wstrb_extended;
  logic [2:0]  cnt_q, cnt_d;
  logic [3:0]  nibble_out;

  for (genvar i = 0; i < 4; i++) begin
    assign wstrb_extended[2*i]   = data_strb[i];
    assign wstrb_extended[2*i+1] = data_strb[i];
  end
  
  
  //Test--------------------------------------------------
  for (genvar i = 0; i < 4; i++) begin
    assign data_pdata[8*i]   = ui_in[i];
    assign data_pdata[8*i+7] = ui_in[i];
  end
  
  assign inst_ready = 1'b1;

  // TODO: Assign to correct output signals
  logic [3:0]  req_data_out;
  logic        req_data_valid, req_data_ready;
  logic [7:0]  req_addr_out;

  logic [31:0] rsp_data_d, rsp_data_q;
  logic        rsp_data_valid, rsp_data_ready;

  logic [31:0] addr_muxed;
  logic        strb_out;

  assign uio_out[3:0] = nibble_out;
  assign uio_out[7:4] = addr_muxed[8 :5];
  assign uo_out [7:4] = addr_muxed[12:9];
  // TODO: assgin the correct write signal from either insn or data
  assign uo_out [3]   = data_write;
  assign uo_out [2]   = strb_out;
  assign uo_out [1]   = req_data_valid;
  assign uo_out [0]   = rsp_data_ready;

  assign rsp_data_d     = ui_in[7:4];
  assign rsp_data_valid = ui_in[1];
  assign req_data_ready = ui_in[0];

  always_comb begin : req_logic
    // Defaults
    next_state  = state;
    shift_reg_d = shift_reg_q;
    strb_reg_d  = strb_reg_q;
    cnt_d       = cnt_q;
    nibble_out  = 4'd0;
    // We do not ack the request by default
    data_qready = 1'b0;

    // TODO: assign it correctly from MUX, temporary connection for synthesis
    req_addr_out = data_qaddr[7:0] | inst_addr[7:0];

    strb_out     = 1'b0;

    if (data_write) begin
      case (state)
        IDLE: begin
          // TODO: assign it correctly from MUX, temporary connection for synthesis
          if (data_qvalid | inst_valid) begin
            // Upon a valid transfer, save the data into reg
            // TODO: assign it correctly from MUX, temporary connection for synthesis
            shift_reg_d = ((data_qdata|inst_data) >> 4);
            strb_reg_d  = (wstrb_extended >> 1);
            // Send out the first piece of data
            nibble_out  = data_qdata[3:0];
            strb_out    = wstrb_extended[0];

            req_data_valid = 1'b1;

            if (req_data_ready) begin
              // The request has been accepted, add counter and move states
              // Count one since we already send one piece out
              cnt_d      = 1'b1;
              next_state = SEND;
            end
          end
        end

        SEND: begin
          nibble_out      = shift_reg_q[3:0];
          req_data_valid  = 1'b1;
          strb_out        = strb_reg_q[0];

          // The request is accepted, move to next 4b data or finish
          if (req_data_ready) begin
            // Ackowledge the request
            data_qready       = 1'b1;

            // Last count, clear and switch back to idle
            if (cnt_q == 3'd7) begin
              next_state      = IDLE;
              cnt_d           = 1'b0;
              shift_reg_d     = '0;
              strb_reg_d      = '0;
            end else begin
              cnt_d           = cnt_q + 1;
              shift_reg_d     = (shift_reg_q >> 4);
              strb_reg_d      = (strb_reg_q  >> 1);
            end
          end
        end
      endcase
    end else begin
      req_data_valid = 1'b1;
      if (req_data_ready) begin
        data_qready  = 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= IDLE;
      shift_reg_q  <= 32'd0;
      cnt_q        <= 3'd0;
      strb_reg_q   <= '0;
    end else begin
      state        <= next_state;
      shift_reg_q  <= shift_reg_d;
      cnt_q        <= cnt_d;
      strb_reg_q   <= strb_reg_d;
    end
  end

endmodule
