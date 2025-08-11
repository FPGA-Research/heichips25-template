`timescale 1ns/1ps

module testbench;

    parameter  T_CLK_HI       = 5ns;                 // set clock high time
    parameter  T_CLK_LO       = 5ns;                 // set clock low time
    localparam T_CLK          = T_CLK_HI + T_CLK_LO; // calculate clock period

    logic       clk;
    logic       rst_n;

    initial begin
        $dumpfile("waveforms.fst");
        $dumpvars(0, testbench);
    end

    initial begin
        // Generating the clock
        do begin
          clk = 1'b1; #T_CLK_HI;
          clk = 1'b0; #T_CLK_LO;
        end while (1);
    end

    initial begin
        // Resetting the design for 1 cycle
        rst_n     = 1'b0;
        #(T_CLK);
        // Release the reset
        rst_n     = 1'b1;
        #(T_CLK);
    end


    logic [7:0] ui_in;
    logic [7:0] uo_out;
    logic [7:0] uio_in;
    logic [7:0] uio_out;
    logic [7:0] uio_oe;
    logic       ena;


    heichips25_snitch_wrapper tt_um_example (
        .ui_in,
        .uo_out,
        .uio_in,
        .uio_out,
        .uio_oe,
        .ena,
        .clk,
        .rst_n
    );

    logic [3:0] chip_qdata, chip_pdata;
    logic       chip_qstrb, chip_write, chip_qlast;
    logic       chip_req_valid, chip_req_ready;
    logic       chip_rsp_valid, chip_rsp_ready;

    assign chip_qdata  = uio_out[3:0];
    assign ui_in[7:4]  = chip_pdata;
    assign chip_write  = uo_out[3];
    assign chip_qstrb  = uo_out[2];

    assign ui_in[3]    = chip_qlast;

    assign chip_req_valid = uo_out[0];
    assign ui_in[0] = chip_req_ready;
    assign chip_rsp_ready = uo_out[1];
    assign ui_in[1] = chip_rsp_valid;


    // TODO: Reconstruct the 32b request

    // TODO: Decomposite to 4b response


    logic [ 9:0] req_addr;
    logic [31:0] req_data;
    logic        req_write, req_wstrb;
    logic        req_valid, req_ready;

    logic [31:0] rsp_data;
    logic        rsp_valid, rsp_ready;

    snitch_vip #(

    ) i_snitch_vip (
        .clk_i       ( clk       ),
        .rst_ni      ( rst_n     ),
        .req_addr_i  ( req_addr  ),
        .req_data_i  ( req_data  ),
        .req_write_i ( req_write ),
        .req_wstrb_i ( req_wstrb ),
        .req_valid_i ( req_valid ),
        .req_ready_o ( req_ready ),
        .rsp_data_o  ( rsp_data  ),
        .rsp_valid_o ( rsp_valid ),
        .rsp_ready_i ( rsp_ready )
    );

    // tc_sram #(
    //     .NumWords  (2**10        ),
    //     .DataWidth (32           ),
    //     .ByteWidth (8            ),
    //     .NumPorts  (1            ),
    //     .Latency   (1            ),
    //     .SimInit   ("none"       )
    // ) i_sim_mem (
    //     .clk_i  (clk_i     ),
    //     .rst_ni (rst_ni    ),
    //     .req_i  (req_valid ),
    //     .we_i   (req_valid ),
    //     .addr_i (req_addr  ),
    //     .wdata_i(req_data  ),
    //     .be_i   (req_wstrb ),
    //     .rdata_o(rsp_data  )
    // );


    // Bit Map of the Signals:
    /// 4-bit data channel
    // data_i[3:0] = ui_in  [7:4]
    // data_o[3:0] = uio_out[3:0]
    // addr_o[3:0] = uio_out[7:4]
    // addr_o[7:4] = uo_out [7:4]
    // write_o     = uo_out [3]
    // strb_o      = uo_out [2]
    // last_i      = ui_in  [3]

    /// request handshaking
    // valid_o     = uo_out[0]
    // ready_i     = ui_in [0]

    /// response handshaking
    // ready_o     = uo_out[1] DO WE NEED IT?
    // valid_i     = ui_in [1]

endmodule
