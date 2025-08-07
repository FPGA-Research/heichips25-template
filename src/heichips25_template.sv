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
  // byte-write enable
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

  // TODO1: Merge the request and response data channel into a single channel

  // TODO2: Add muxing between instruction and data ports

  // TODO3: Add the FIFO to breakdown the 32b access to 4b
    
    assign uo_out  = count;
    assign uio_out = count;
    assign uio_oe  = '1;

endmodule
