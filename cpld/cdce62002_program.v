/*
(c) 2010 Eli Billauer

From http://billauer.co.il/blog/2010/10/cdce62002-pll-vco-registers-ram-eeprom/
*/


module cdce62002
  (
   input          clk, // Maximum 40 MHz
   input      reset, // Active high

   output     busy,

   input          send_data,
   output reg     spi_clk, spi_le, spi_mosi,
   input          spi_miso,  // Never used

   // The names below match those used in pages 22-24 of the datasheet

   input  INBUFSELX,
   input  INBUFSELY,
   input  REFSEL,
   input  AUXSEL,
   input  ACDCSEL,
   input  TERMSEL,
   input [3:0]  REFDIVIDE,
   input [1:0]  LOCKW,
   input [3:0]  OUT0DIVRSEL,
   input [3:0]  OUT1DIVRSEL,
   input   HIPERFORMANCE,
   input   OUTBUFSEL0X,
   input   OUTBUFSEL0Y,
   input   OUTBUFSEL1X,
   input   OUTBUFSEL1Y,

   input  SELVCO,
   input [7:0]  SELINDIV,
   input [1:0]  SELPRESC,
   input [7:0]  SELFBDIV,
   input [2:0]  SELBPDIV,
   input [3:0]  LFRCSEL
   );

   reg [7:0]      out_pointer;
   reg        active;

   wire [255:0]   data_out;
   wire [255:0]   le_out;
   wire [27:0]    word0, word1;
   wire [27:0]    ones = 28'hfffffff, zeros = 28'h0000000;

   assign busy = (out_pointer != 8'b00000000);
	wire done = (out_pointer[7] == 1'b1);

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
               word0, 4'b0000, 2'd0,
               word1, 4'b0001, 2'd0,
               zeros[27:0], 4'b1111, 4'd0,
               64'd0 // Dwell a bit in power down
               };

   assign le_out = {
             ones[27:0], ones[3:0], 2'd0,
             ones[27:0], ones[3:0], 2'd0,
             ones[27:0], ones[3:0], 4'd0,
             64'd0 // Dwell a bit in power down
            };

   assign word0[0] = INBUFSELX;
   assign word0[1] = INBUFSELY;
   assign word0[2] = REFSEL;
   assign word0[3] = AUXSEL;
   assign word0[4] = ACDCSEL;
   assign word0[5] = TERMSEL;
   assign word0[9:6] = REFDIVIDE;
   assign word0[10] = 0; // TI trashed external feedback
   assign word0[12:11] = 0; // TI's test bits
   assign word0[14:13] = LOCKW;
   assign word0[18:15] = OUT0DIVRSEL;
   assign word0[22:19] = OUT1DIVRSEL;
   assign word0[23] = HIPERFORMANCE;
   assign word0[24] = OUTBUFSEL0X;
   assign word0[25] = OUTBUFSEL0Y;
   assign word0[26] = OUTBUFSEL1X;
   assign word0[27] = OUTBUFSEL1Y;

   assign word1[0] = SELVCO;
   assign word1[8:1] = SELINDIV;
   assign word1[10:9] = SELPRESC;
   assign word1[18:11] = SELFBDIV;
   assign word1[21:19] = SELBPDIV;
   assign word1[25:22] = LFRCSEL;
   assign word1[27:26] = 2'b10; // Read only bits
endmodule
