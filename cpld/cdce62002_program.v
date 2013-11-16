/*
(c) 2010 Eli Billauer

From http://billauer.co.il/blog/2010/10/cdce62002-pll-vco-registers-ram-eeprom/
*/


module cdce62002
  (
   input          clk, // Maximum 40 MHz
   input      reset, // Active high

   output reg     active,

   input          send_data,
   output reg     spi_clk, spi_le, spi_mosi,
   input          spi_miso
   );

   reg [8:0]      out_pointer;

   wire [511:0]   data_out;
   wire [511:0]   le_out;
   wire [27:0]    word0, word1;
   wire [27:0]    ones = 28'hfffffff, zeros = 28'h0000000;
   wire busy;

   assign busy = (out_pointer != 9'b00000000);
	wire done = (out_pointer[8] == 1'b1);

   // "active" is necessary because we don't rely on getting a proper
   // reset signal, and out_pointer is subject to munching by the
   // synthesizer, which may result in nasty things during wakeup

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
      end
		spi_clk <= !spi_clk;
	end

   assign data_out = {
             32'h61003bf2, 4'd0,
             32'h60003bf2, 4'd0,
             32'h61003bf2, 4'd0,
             32'h00000000, 4'd0,
             32'h8383E001, 4'd0,
             32'h55D00080, 8'd0
             };

   assign le_out = {
           32'hffffffff, 4'd0,
           32'hffffffff, 4'd0,
           32'hffffffff, 4'd0,
           32'h00000000, 4'd0,
           32'hffffffff, 4'd0,
           32'hffffffff, 8'd0 };
endmodule
