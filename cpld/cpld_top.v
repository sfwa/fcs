module cpld_top(
	/* BANK 1 -- 3V3 */
	output spi_flash_clk,
	output spi_flash_mosi,
	input spi_flash_miso,
	output spi_flash_cs_INV,
	output pll_spi_clk,
	output pll_spi_mosi,
	input pll_spi_miso,
	output pll_spi_cs_INV,
	output smbus_clk, /* open drain */
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
	output ucd9222_rst,
	input pg_ucd9222,
	input pg_cvdd,
	input pg_1v0,
	output camera_trigger,
	output dsp_ext_spi_en,
	output dsp_ext_uart_en,
	output cpu_ext_uart_en,
	output ioboard_2_reset_out,
	input ioboard_2_reset_in,
	output ioboard_1_reset_out,
	input ioboard_1_reset_in,
	output cpu_usbhub_reset_INV,
	input cell_wake_INV,
	output cell_gps_en_INV,
	output cell_disable_INV,
	output led[3:0],
	inout gpio[4:0],
	/* BANK 2 -- DSP 1V8 */
	input dsp_spi_clk,
	input dsp_spi_mosi,
	output dsp_spi_miso,
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
	inout dsp_gpio[19:0],
	input dsp_ext_uart_tx,
	input dsp_ext_uart0_int_INV,
	input dsp_ext_uart1_int_INV,
	output dsp_usb_reset_INV,
	input dsp_usb_irq_INV,
	output dsp_usb_dack,
	output dsp_usb_eot,
	input dsp_usb_dreq,
	input dsp_int_uart0_tx,
	output dsp_int_uart0_rx,
	input dsp_int_uart1_tx,
	output dsp_int_uart1_rx,
	output dsp_ext_uart_rx,
	/* BANK 3 -- 3V3 */
	output ext_spi_clk,
	output ext_spi_mosi,
	input ext_spi_miso,
	output ext_spi_cs_INV,
	output ioboard_uart0_tx,
	input ioboard_uart0_rx,
	output ioboard_uart1_tx,
	input ioboard_uart1_rx,
	output ext_uart0_tx,
	input ext_uart0_rx,
	output ext_uart1_tx,
	input ext_uart1_rx,
	/* BANK 4 -- CPU 1V8 */
	input cpu_spi0_1v8_mosi,
	output cpu_spi0_1v8_miso,
	input cpu_spi0_1v8_clk,
	input cpu_spi0_1v8_cs_INV,
	input cpu_spi1_1v8_mosi,
	output cpu_spi1_1v8_miso,
	input cpu_spi1_1v8_clk,
	input cpu_spi1_1v8_cs_INV,
	input cpu_resetout,
	output cpu_pmic_reset_INV,
	output cpu_pmic_pwron,
	output cpu_reset_INV,
	output cpu_wreset_INV,
	output cpu_bootmode[5:0],
	inout cpu_gpio[23:0],
	input cpu_ext_uart0_tx,
	output cpu_ext_uart0_rx,
	input cpu_ext_uart1_tx,
	output cpu_ext_uart1_rx
);

wire osc_clk, osc_en;

assign osc_en = 1'b1;
altufm_osc0_altufm_osc_1p3 int_osc(.osc(osc_clk), .oscena(osc_en));

endmodule

/**************************************************************************************************************************************************/
/* Megafunction for Internal oscillator */

`timescale 1 ps / 1 ps
module altufm_osc0_altufm_osc_1p3
	( 
	osc,
	oscena) /* synthesis synthesis_clearbox=1 */;
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
