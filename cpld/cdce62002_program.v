/*
(c) 2010 Eli Billauer

From http://billauer.co.il/blog/2010/10/cdce62002-pll-vco-registers-ram-eeprom/
*/


module cdce62002 (
    input clk,
    input reset,
    output reg active,
    input send_data,
    output reg spi_clk, spi_le, spi_mosi,
    input spi_miso
);

reg[8:0] out_pointer = 9'b0;

/*
These parameters appear to yield a fairly stable clock signal, but the PLL
doesn't seem to lock properly:
bb870061
54220080

These parameters are stable enough for the DSP to boot and complete testing,
but the PLL doesn't lock if 1v8 is high:
b7870061
54200080

These parameters are also suggested by the CDCE62002 tool given our component
values and tolerances:
b782a001
56320080
*/

/*
Reads bottom-to-top. Write config registers, wait a while, trigger a
calibration, then wait a bit longer
*/
wire[511:0] data_out = {
    32'h00000000, 4'd0,
    32'h610233f2, 4'd0,
    32'h600233f2, 4'd0,
    32'h610233f2, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'hb7870061, 4'd0,
    32'h54200080, 8'd0
};

wire[511:0] le_out  = {
    32'h00000000, 4'd0,
    32'hffffffff, 4'd0,
    32'hffffffff, 4'd0,
    32'hffffffff, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'h00000000, 4'd0,
    32'hffffffff, 4'd0,
    32'hffffffff, 8'd0
};

wire busy = (out_pointer != 9'b00000000);
wire done = (out_pointer[8] == 1'b1);

always @(posedge clk) begin
    if (done) begin
        active <= 1'b0;
    end else if (reset) begin
        out_pointer <= 1'b0;
        active <= 1'b0;
    end else if (send_data & !busy) begin
        out_pointer <= 1'b1;
        active <= 1'b1;
    end else if (spi_clk && busy) begin
        out_pointer <= out_pointer + 1'b1;
        active <= active;
    end else begin
        out_pointer <= out_pointer;
        active <= active;
    end
end

always @(posedge clk) begin
    if (spi_clk) begin
        spi_mosi <= data_out[out_pointer];
        spi_le <= !(le_out[out_pointer] && active);
    end
    spi_clk <= !spi_clk;
end

endmodule
