// SPDX-FileCopyrightText: © 2025 XXX Authors
// SPDX-License-Identifier: Apache-2.0

// Adapted from the Tiny Tapeout template

`default_nettype none

module heichips25_snitch_wrapper (
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

  // REMOVE THIS LATER IF YOU DECIDE TO USE THEM 
  // ADDING THIS TO REMOVE SYNTH CHECK ERROR
  assign uio_oe       = 8'hFF;

  snitch #(
    .BootAddr ( BootAddr ),
    .MTVEC    ( BootAddr ),
    .RVE      ( 1'b0     ),
    .RVM      ( 1'b1     )
  ) i_snitch (
    .clk_i              ( clk           ),
    .rst_i              ( !rst_n        ),
    .hart_id_i          ( '0            ),
    .inst_addr_o        ( inst_addr     ),
    .inst_data_i        ( inst_data     ),
    .inst_valid_o       ( inst_valid    ),
    .inst_ready_i       ( inst_ready    ),
    .acc_qaddr_o        (               ),
    .acc_qid_o          (               ),
    .acc_qdata_op_o     (               ),
    .acc_qdata_arga_o   (               ),
    .acc_qdata_argb_o   (               ),
    .acc_qdata_argc_o   (               ),
    .acc_qvalid_o       (               ),
    .acc_qready_i       ( '0            ),
    .acc_pdata_i        ( '0            ),
    .acc_pid_i          ( '0            ),
    .acc_perror_i       ( '0            ),
    .acc_pvalid_i       ( '0            ),
    .acc_pready_o       (               ),
    .acc_pwrite_i       ( '0            ),
    .data_qaddr_o       ( data_qaddr    ),
    .data_qwrite_o      ( data_write    ),
    .data_qamo_o        (               ),
    .data_qdata_o       ( data_qdata    ),
    .data_qstrb_o       ( data_strb     ),
    .data_qid_o         (               ),
    .data_qvalid_o      ( data_qvalid   ),
    .data_qready_i      ( data_qready   ),
    .data_pdata_i       ( data_pdata    ),
    .data_perror_i      ( '0            ),
    .data_pid_i         ( '0            ),
    .data_pvalid_i      ( data_pvalid   ),
    .data_pready_o      ( data_pready   ),
    .wake_up_sync_i     ( wake_up_sync  )
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

  typedef struct packed {
    logic  [7:0] addr;
    logic [31:0] data;
    logic        write;
    logic  [3:0] strb;
  } mem_req_t;

  mem_req_t lsu_req, inst_req, muxed_req;
  logic     muxed_valid, muxed_ready;

  mem_req_t postreg_req;
  logic     postreg_valid, postreg_ready;

  assign lsu_req = '{
    addr : data_qaddr[9:2],
    data : data_qdata,
    strb : data_strb,
    write: data_write
  };

  assign inst_req = '{
    addr : inst_addr[9:2],
    data : inst_data,
    strb : '0,
    write: 1'b0
  };

  logic rr_sel;

  for (genvar i = 0; i < 4; i++) begin
    assign wstrb_extended[2*i]   = postreg_req.strb[i];
    assign wstrb_extended[2*i+1] = postreg_req.strb[i];
  end

  // assign inst_ready = 1'b1;

  // TODO: Assign to correct output signals
  logic [3:0]  req_data_out;
  logic        req_data_valid, req_data_ready;
  logic [7:0]  req_addr_out;

  logic [31:0] rsp_data_d, rsp_data_q;
  logic        rsp_data_valid, rsp_data_ready;
  logic        rsp_data_last;

  // logic [31:0] addr_muxed;
  logic        strb_out;

  logic        target_sel_d, target_sel_q;

  assign uio_out[3:0] = req_data_out;
  assign uio_out[7:4] = postreg_req.addr[7:4];
  assign uo_out [7:4] = postreg_req.addr[3:0];
  // TODO: assgin the correct write signal from either insn or data
  assign uo_out [3]   = postreg_req.write;
  assign uo_out [2]   = strb_out;
  assign uo_out [1]   = rsp_data_ready;
  assign uo_out [0]   = req_data_valid;

  assign rsp_data_last  = ui_in[3];
  assign wake_up_sync   = ui_in[2];
  assign rsp_data_valid = ui_in[1];
  assign req_data_ready = ui_in[0];

  typedef enum logic {
    PARTIAL, DONE
  } rsp_fsm_e;

  rsp_fsm_e rsp_state_d, rsp_state_q;
  logic[31:0] prereg_pdata;
  logic       prereg_pvalid, prereg_pready;

  always_comb begin : rsp_logic
    rsp_data_d  = rsp_data_q;
    rsp_state_d = rsp_state_q;
    inst_ready  = 1'b0;
    inst_data   = '0;
    prereg_pdata  = '0;
    prereg_pvalid = 1'b0;

    rsp_data_ready = 1'b0;

    case (rsp_state_q)
      PARTIAL: begin
        // MSB to store the data
        if (rsp_data_valid) begin
          rsp_data_d = rsp_data_q << 4;
          rsp_data_d[3:0] = ui_in[7:4];
          // ackowledge the transaction
          rsp_data_ready = 1'b1;

          if (rsp_data_last) begin
            // This signal will be given from the converter in eFPGA
            rsp_state_d = DONE;
          end
        end
      end

      DONE: begin
        if (rsp_data_valid) begin
          if (target_sel_q == 0) begin
            // 0 is pointing to the instruction
            inst_data   = rsp_data_q;
            // Half handshaking on instruction side
            inst_ready  = 1'b1;

            rsp_data_d  = '0;
            rsp_state_d = PARTIAL;
          end else begin
            // 1 is pointing to the LSU
            prereg_pdata  = rsp_data_q;
            prereg_pvalid = 1'b1;

            if (prereg_pready) begin
              // Accepted, clean and return
              rsp_data_d = '0;
              rsp_state_d = PARTIAL;
            end
          end
        end
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin : fsm_rsp
    if (!rst_n) begin
      rsp_state_q  <= PARTIAL;
      rsp_data_q   <= '0;
      target_sel_q <= '0;
    end else begin
      rsp_state_q  <= rsp_state_d;
      rsp_data_q   <= rsp_data_d;
      target_sel_q <= target_sel_d;
    end
  end

  spill_register #(
    .T      (logic[31:0]    )
  ) i_rsp_register (
    .clk_i  (clk            ),
    .rst_ni (rst_n          ),
    .data_i (prereg_pdata   ),
    .valid_i(prereg_pvalid  ),
    .ready_o(prereg_pready  ),
    .data_o (data_pdata     ),
    .valid_o(data_pvalid    ),
    .ready_i(data_pready    )
  );

  // instruction does not have a full HS
  // this bit is not used
  logic inst_req_ready;

  // TODO: Add the arbiter for two ports
  rr_arb_tree #(
    .NumIn     ( 2         ),
    .DataType  ( mem_req_t ),
    .ExtPrio   ( 1'b0      ),
    .AxiVldRdy ( 1'b1      ),
    .LockIn    ( 1'b1      )
  ) i_req_arb (
    .clk_i   ( clk                  ),
    .rst_ni  ( rst_n                ),
    .flush_i ( '0                   ),
    .rr_i    ( '0                   ),
    .req_i   ( {data_qvalid,  inst_valid}     ),
    .gnt_o   ( {data_qready,  inst_req_ready} ),
    .data_i  ( {lsu_req,      inst_req}       ),
    .req_o   ( muxed_valid                    ),
    .gnt_i   ( muxed_ready                    ),
    .data_o  ( muxed_req                      ),
    .idx_o   ( rr_sel                         )
  );

  spill_register #(
    .T      (mem_req_t      )
  ) i_req_register (
    .clk_i  (clk            ),
    .rst_ni (rst_n          ),
    .data_i (muxed_req      ),
    .valid_i(muxed_valid    ),
    .ready_o(muxed_ready    ),
    .data_o (postreg_req    ),
    .valid_o(postreg_valid  ),
    .ready_i(postreg_ready  )
  );

  // spill_register #(
  //   .T(tcdm_master_req_t)
  // ) i_tcdm_master_req_register (
  //   .clk_i  (clk_i                          ),
  //   .rst_ni (rst_ni                         ),
  //   .data_i (prereg_tcdm_master_req[h]      ),
  //   .valid_i(prereg_tcdm_master_req_valid[h]),
  //   .ready_o(prereg_tcdm_master_req_ready[h]),
  //   .data_o (tcdm_master_req_o[h]           ),
  //   .valid_o(tcdm_master_req_valid_o[h]     ),
  //   .ready_i(tcdm_master_req_ready_i[h]     )
  // );

  always_comb begin : req_logic
    // Defaults
    next_state      = state;
    shift_reg_d     = shift_reg_q;
    strb_reg_d      = strb_reg_q;
    cnt_d           = cnt_q;
    target_sel_d    = target_sel_q;

    req_data_out    = 4'd0;
    req_data_valid  = 1'b0;
    // addr_muxed      = '0;

    // TODO: connect the sel_d signal to the rr_arb output

    // We do not ack the request by default
    postreg_ready = 1'b0;

    // TODO: assign it correctly from MUX, temporary connection for synthesis
    req_addr_out = postreg_req.addr;

    strb_out     = 1'b0;

    if (postreg_req.write) begin
      case (state)
        IDLE: begin
          // TODO: assign it correctly from MUX, temporary connection for synthesis
          if (postreg_valid) begin
            // Upon a valid transfer, save the data into reg
            // TODO: assign it correctly from MUX, temporary connection for synthesis
            shift_reg_d = ((postreg_req.data) >> 4);
            strb_reg_d  = (wstrb_extended >> 1);
            // Send out the first piece of data
            req_data_out  = postreg_req.data[3:0];
            strb_out    = wstrb_extended[0];

            req_data_valid = 1'b1;

            // Select the address from the mux
            // addr_muxed = postreg_req.addr;

            if (req_data_ready) begin
              // The request has been accepted, add counter and move states
              // Count one since we already send one piece out
              cnt_d      = 1'b1;
              next_state = SEND;
            end
          end
        end

        SEND: begin
          req_data_out    = shift_reg_q[3:0];
          req_data_valid  = 1'b1;
          strb_out        = strb_reg_q[0];

          // The request is accepted, move to next 4b data or finish
          if (req_data_ready) begin
            // Do not ack the req until all have been translated
            postreg_ready       = 1'b0;

            // Last count, clear and switch back to idle
            if (cnt_q == 3'd7) begin
              next_state      = IDLE;
              cnt_d           = 1'b0;
              shift_reg_d     = '0;
              strb_reg_d      = '0;
              postreg_ready     = 1'b1;
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
        postreg_ready  = 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin : fsm_req
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
