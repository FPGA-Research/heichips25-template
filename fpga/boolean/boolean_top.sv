module boolean_top (
    input  logic        clk, // 100 MHz

    input  logic [15:0] sw,
    output logic [15:0] led,
    input  logic [3:0]  btn,
    
    // HDMI
    //output  logic [2:0]  hdmi_tx_p,
    //output  logic [2:0]  hdmi_tx_n,
    //output  logic        hdmi_clk_n, hdmi_clk_p,

    // RGB LEDs
    output logic  [2:0]  RGB0, RGB1  
);

    //logic clk;
    logic rst_n;
    logic ena;
    logic [7:0] ui_in;
    logic [7:0] uio_in;
    logic [7:0] uo_out;
    logic [7:0] uio_out;
    logic [7:0] uio_oe;
    
    heichips25_template heichips25_template (
        .ui_in  (ui_in),    // Dedicated inputs
        .uo_out (uo_out),   // Dedicated outputs
        .uio_in (uio_in),   // IOs: Input path
        .uio_out(uio_out),  // IOs: Output path
        .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
        .ena    (ena),      // enable - goes high when design is selected
        .clk    (clk),      // clock
        .rst_n  (rst_n)     // not reset
    );
    
    // Assignments

    assign ui_in = sw[7:0];
    assign uio_in = sw[15:8];
    
    assign led[7:0] = uo_out;
    assign led[15:8] = uio_out;
    
    assign ena = 1'b1;
    assign rst_n = !btn[0];

endmodule
