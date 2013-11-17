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

reg[8:0] out_pointer;

wire[511:0] data_out = {
    32'h61003bf2, 4'd0,
    32'h60003bf2, 4'd0,
    32'h61003bf2, 4'd0,
    32'h00000000, 4'd0,
    32'h8383E001, 4'd0,
    32'h55D00080, 8'd0
};

wire[511:0] le_out  = {
    32'hffffffff, 4'd0,
    32'hffffffff, 4'd0,
    32'hffffffff, 4'd0,
    32'h00000000, 4'd0,
    32'hffffffff, 4'd0,
    32'hffffffff, 8'd0
};

wire busy = (out_pointer != 9'b00000000);
wire done = (out_pointer[8] == 1'b1);

always @(posedge clk) begin
    if (reset | done) begin
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
    end else begin
        spi_clk <= !spi_clk;
    end
end

endmodule
