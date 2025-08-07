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

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[7:1], uio_in[7:0]};

  logic [7:0] count;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      count <= '0;
    end else begin
      if (ui_in[0]) begin
        count <= count + 1;
      end
    end
  end

  // Instruction interface (should be muxed with data)
  logic [31:0] inst_addr, inst_data;
  logic inst_valid, inst_ready;

  // Data interface (q means request, p means response)
  logic [31:0] data_qaddr, data_qdata, data_pdata;
  logic [3 :0] data_strb;
  logic data_qvalid, data_qready, data_pvalid, data_pready;

  // TODO (Diyou): Assign correct BootAddr
  // TODO (Diyou): Which extension to enable/disable?

  snitch #(
    .BootAddr ( BootAddr ),
    .MTVEC    ( MTVEC    ),
    .RVE      ( RVE      ),
    .RVM      ( RVM      )
  ) i_snitch (
    .clk_i            ( clk           ),
    .rst_ni           ( rst_n         ),
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
    .data_qamo_o      ( '0            ),
    .data_qdata_o     ( data_qdata    ),
    .data_qstrb_o     ( data_strb     ),
    .data_qid_o       ( '0            ),
    .data_qvalid_o    ( data_qvalid   ),
    .data_qready_i    ( data_qready   ),
    .data_pdata_i     ( data_pdata    ),
    .data_perror_i    ( '0            ),
    .data_pid_i       ( '0            ),
    .data_pvalid_i    ( data_pvalid   ),
    .data_pready_o    ( data_pready   ),
    .wake_up_sync_i   ( wake_up_sync  ),
    .fpu_rnd_mode_o   (               ),
    .fpu_status_i     ( '0            ),
    .core_events_o    (               )
  );

  // === TODO3: FIFO to serialize 32-bit to 4-bit ===

  typedef enum logic [1:0] {
    IDLE, LOAD, SEND
  } fsm_state_e;

  fsm_state_e state, next_state;
  logic [31:0] shift_reg_q, shift_reg_d;
  logic [2:0]  cnt_q, cnt_d;
  logic [3:0]  nibble_out;

  always_comb begin
    // Defaults
    next_state  = state;
    shift_reg_d = shift_reg_q;
    cnt_d       = cnt_q;
    nibble_out  = 4'd0;

    case (state)
      IDLE: begin
        if (data_qvalid) begin
          next_state = LOAD;
        end
      end

      LOAD: begin
        shift_reg_d = data_qdata;
        cnt_d       = 3'd0;
        next_state  = SEND;
      end

      SEND: begin
        nibble_out  = shift_reg_q[3:0];
        shift_reg_d = shift_reg_q >> 4;
        cnt_d       = cnt_q + 1;
        data_pready = 1'b0;
        data_pvalid = 1'b1;


        if (cnt_q == 3'd7)
          next_state = IDLE;
          data_pready = 1'b1;
          data_pvalid = 1'b0;
        end
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= IDLE;
      shift_reg_q  <= 32'd0;
      cnt_q        <= 3'd0;
    end else begin
      state        <= next_state;
      shift_reg_q  <= shift_reg_d;
      cnt_q        <= cnt_d;
    end
  end

endmodule
