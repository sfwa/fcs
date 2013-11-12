/*
Copyright (c) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


module cpld_top(
	/* BANK 1 -- 3V3 */
	output reg spi_flash_clk,
	output reg spi_flash_mosi,
	input spi_flash_miso,
	output reg spi_flash_cs_INV,
	output pll_spi_clk,
	output pll_spi_mosi,
	input pll_spi_miso,
	output pll_spi_cs_INV,
	inout smbus_clk, /* open drain */
	inout smbus_data, /* open drain */
	input smbus_alert,
	output smbus_cntrl,
	output en_1v5,
	output en_vtt,
	input pg_1v5,
	input pg_vtt,
	output pll_en,
	input pll_locked,
	output en_1v8,
	input pg_5v,
	input pg_3v3,
	output en_cvdd,
	output en_1v0,
	output ucd9222_rst_INV,
	input pg_ucd9222,
	input pg_cvdd,
	input pg_1v0,
	output reg camera_trigger,
	output reg dsp_ext_spi_en,
	output reg dsp_ext_uart_en,
	output reg cpu_ext_uart_en,
	output reg ioboard_2_reset_out,
	input ioboard_2_reset_in,
	output reg ioboard_1_reset_out,
	input ioboard_1_reset_in,
	output reg cpu_usbhub_reset_INV,
	input cell_wake_INV,
	output reg cell_gps_en_INV,
	output reg cell_disable_INV,
	output reg[3:0] led,
	inout[4:0] gpio,
	/* BANK 2 -- DSP 1V8 */
	input dsp_spi_clk,
	input dsp_spi_mosi,
	output reg dsp_spi_miso,
	input dsp_spi_cs0_INV,
	input dsp_spi_cs1_INV,
	inout dsp_i2c_1v8_scl, /* open drain */
	inout dsp_i2c_1v8_sda, /* open drain */
	input dsp_booted,
	input dsp_hout,
	input dsp_resetstat,
	output dsp_por_INV,
	output dsp_resetfull_INV,
	output dsp_reset_INV,
	output dsp_lresetnmien_INV,
	output dsp_lreset_INV,
	output dsp_nmi_INV,
	output dsp_vid_oe_INV,
	output dsp_coresel0,
	output dsp_coresel1,
	inout reg[19:0] dsp_gpio,
	input dsp_ext_uart_tx,
	input dsp_ext_uart0_int_INV,
	input dsp_ext_uart1_int_INV,
	output reg dsp_usb_reset_INV,
	input dsp_usb_irq_INV,
	output reg dsp_usb_dack,
	output reg dsp_usb_eot,
	input dsp_usb_dreq,
	input dsp_int_uart0_tx,
	output reg dsp_int_uart0_rx,
	input dsp_int_uart1_tx,
	output reg dsp_int_uart1_rx,
	output reg dsp_ext_uart_rx,
	/* BANK 3 -- 3V3 */
	output reg ext_spi_clk,
	output reg ext_spi_mosi,
	input ext_spi_miso,
	output reg ext_spi_cs_INV,
	output reg ioboard_uart0_tx,
	input ioboard_uart0_rx,
	output reg ioboard_uart1_tx,
	input ioboard_uart1_rx,
	output reg ext_uart0_tx,
	input ext_uart0_rx,
	output reg ext_uart1_tx,
	input ext_uart1_rx,
	/* BANK 4 -- CPU 1V8 */
	input cpu_spi0_1v8_mosi,
	output reg cpu_spi0_1v8_miso,
	input cpu_spi0_1v8_clk,
	input cpu_spi0_1v8_cs_INV,
	input cpu_spi1_1v8_mosi,
	output reg cpu_spi1_1v8_miso,
	input cpu_spi1_1v8_clk,
	input cpu_spi1_1v8_cs_INV,
	input cpu_resetout,
	output cpu_pmic_reset_INV,
	output cpu_pmic_pwron,
	output reg cpu_reset_INV,
	output reg cpu_wreset_INV,
	output[5:0] cpu_bootmode, /* open drain */
	inout[23:0] cpu_gpio,
	input cpu_ext_uart0_tx,
	output reg cpu_ext_uart0_rx,
	input cpu_ext_uart1_tx,
	output reg cpu_ext_uart1_rx
);

/* Defaults */
assign dsp_i2c_1v8_scl = 1'bz;
assign dsp_i2c_1v8_sda = 1'bz;
assign cpu_gpio[23:0] = 24'bz;

wire osc_clk, osc_clk_100us, dsp_enable, cpu_enable, io1_enable, io2_enable,
     dsp_bootmode_en, pg_ddr3, pg_1v8, sys_enable;
wire[15:0] dsp_bootmode;
reg[8:0] osc_clk_scaler;

/*
User flash module oscillator output -- anywhere from 3.3-5.5MHz, used for
power sequencing timers etc
*/
altufm_osc0_altufm_osc_1p3 int_osc(
	.osc(osc_clk),
	.oscena(1'b1)
);

/* Generate osc_clk_100us from osc_clk / 512 */
assign osc_clk_100us = osc_clk_scaler[8];
always @(posedge osc_clk) begin
	osc_clk_scaler <= osc_clk_scaler + 8'b000000001;
end

/*
Global system enable -- wait until the board power supplies are good
*/
assign sys_enable = pg_5v & pg_3v3;

assign gpio[4] = 1'bz;
assign cpu_enable = gpio[4];
assign dsp_enable = gpio[4];

/*
DSP sequencer -- handles power on/off for the DSP and associated peripherals
*/
assign en_vtt = en_1v5; /* FIXME? */
assign pg_ddr3 = pg_1v5 & pg_vtt; /* FIXME? */
assign pg_1v8 = en_1v8; /* FIXME */
assign ucd9222_rst_INV = !sys_enable;
c66x_sequencer dsp_seq(
    .sysclk(osc_clk_100us),
    .enable(dsp_enable & sys_enable),
    .cvdd_good(pg_cvdd),
    .cvdd1_good(pg_1v0),
    .dvdd18_good(pg_1v8),
    .dvdd15_good(pg_ddr3),
    .pll_locked(pll_locked),
    .resetstat_INV(dsp_resetstat),
    .cvdd_en(en_cvdd),
    .cvdd1_en(en_1v0),
    .dvdd18_en(en_1v8),
    .dvdd15_en(en_1v5),
    .pll_en(pll_en),
    .por_INV(dsp_por_INV),
    .reset_INV(dsp_reset_INV),
    .resetfull_INV(dsp_resetfull_INV),
    .bootmode_en(dsp_bootmode_en),
    .bootmode(dsp_bootmode)
);

/*
Handle DSP bootmode mux -- if dsp_bootmode_en asserted, then pass the bootmode
signals from the C66x sequencer through to the GPIOs.

If it's not asserted, we can use them however we like.
*/
always @(*) begin
	dsp_gpio[15:0] = 16'b0;
	if (dsp_bootmode_en) begin
		dsp_gpio = dsp_bootmode;
	end else begin
		dsp_gpio = 16'bz;
	end
end

/*
CPU sequencer -- handle power on/off for the CPU board
*/
exynos4412_sequencer cpu_seq(
	.sysclk(osc_clk_100us),
    .enable(cpu_enable & sys_enable),
    .cpu_pmic_pwron(cpu_pmic_pwron),
    .cpu_pmic_reset_INV(cpu_pmic_reset_INV),
    .cpu_bootmode(cpu_bootmode)
);

/*
Board setup -- configure a PMBus pass-through from the CPLD GPIO header to the
UCD9222 DSP core power controller
*/
assign gpio[2] = smbus_alert ? 1'b1 : 1'b0;
assign gpio[3] = 1'bz;
assign smbus_cntrl = gpio[3] ? 1'b1 : 1'b0;
pmbus_passthrough ucd9222_interface(
    .reset(!sys_enable),
    .sysclk(osc_clk),
    .master_scl(gpio[0]),
    .master_sda(gpio[1]),
    .slave_scl(smbus_clk),
    .slave_sda(smbus_data),
    .sda_direction_tap()
);

/*
Program the CDCE62002 with preset configuration words:
REGISTERS
0	55200080
1	8389A061
2	00000002
*/
cdce62002 clkgen_interface(
   .clk(osc_clk),
   .reset(!sys_enable),
   .busy(),
   .send_data(sys_enable),
   .spi_clk(pll_spi_clk),
   .spi_le(pll_spi_cs_INV),
   .spi_mosi(pll_spi_mosi),
   .spi_miso(pll_spi_miso),

   /* The names below match those used in pages 22-24 of the datasheet*/
   .INBUFSELX(1'b0),
   .INBUFSELY(1'b0),
   .REFSEL(1'b0),
   .AUXSEL(1'b1),
   .ACDCSEL(1'b0),
   .TERMSEL(1'b0),
   .REFDIVIDE(4'b0000),
   .LOCKW(2'b00),
   .OUT0DIVRSEL(4'b0100),
   .OUT1DIVRSEL(4'b1010),
   .HIPERFORMANCE(1'b0),
   .OUTBUFSEL0X(1'b1),
   .OUTBUFSEL0Y(1'b0),
   .OUTBUFSEL1X(1'b1),
   .OUTBUFSEL1Y(1'b0),

   .SELVCO(1'b0),
   .SELINDIV(8'b00000011),
   .SELPRESC(2'b01),
   .SELFBDIV(8'b00010011),
   .SELBPDIV(3'b111),
   .LFRCSEL(4'b0000)
);

endmodule

/****************************************************************************/
/* Megafunction for Internal oscillator */

`timescale 1 ps / 1 ps
module altufm_osc0_altufm_osc_1p3(osc, oscena);
	output   osc;
	input   oscena;

	wire  wire_maxii_ufm_block1_osc;

	maxii_ufm   maxii_ufm_block1
	(
	.arclk(1'b0),
	.ardin(1'b0),
	.arshft(1'b0),
	.bgpbusy(),
	.busy(),
	.drclk(1'b0),
	.drdout(),
	.drshft(1'b0),
	.osc(wire_maxii_ufm_block1_osc),
	.oscena(oscena),
	.drdin(1'b0),
	.erase(1'b0),
	.program(1'b0),
	.ctrl_bgpbusy(),
	.devclrn(),
	.devpor(),
	.sbdin(),
	.sbdout()
	);
	defparam
		maxii_ufm_block1.address_width = 9,
		maxii_ufm_block1.osc_sim_setting = 300000,
		maxii_ufm_block1.lpm_type = "maxii_ufm";
	assign
		osc = wire_maxii_ufm_block1_osc;
endmodule //altufm_osc0_altufm_osc_1p3
