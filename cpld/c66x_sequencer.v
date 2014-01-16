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

module c66x_sequencer(
    input sysclk,
    input enable,
    input cvdd_good,
    input cvdd1_good,
    input dvdd18_good,
    input dvdd15_good,
    input pll_locked,
    input resetstat_INV,
    output reg cvdd_en,
    output reg cvdd1_en,
    output reg dvdd18_en,
    output reg dvdd15_en,
    output reg pll_en,
    output reg por_INV,
    output reg reset_INV,
    output reg resetfull_INV,
    output reg vid_oe_INV,
    output reg dsp_bank_en,
    output reg bootmode_en,
    output reg[15:0] bootmode,
    output reg[3:0] state,
    output pll_spi_clk,
    output pll_spi_cs_INV,
    output pll_spi_mosi
);

reg[1:0] cvdd_good_debounce;
reg[1:0] cvdd1_good_debounce;
reg[1:0] dvdd18_good_debounce;
reg[1:0] dvdd15_good_debounce;
reg[3:0] pll_locked_debounce;
reg[1:0] resetstat_INV_debounce;

wire cvdd_ok = (cvdd_good_debounce == 2'b11);
wire cvdd1_ok = (cvdd1_good_debounce == 2'b11);
wire dvdd18_ok = (dvdd18_good_debounce == 2'b11);
wire dvdd15_ok = (dvdd15_good_debounce == 2'b11);
wire pll_ok = (pll_locked_debounce == 4'b1111);
wire resetstat_ok = (resetstat_INV_debounce == 2'b11);

wire clkgen_busy;
reg clkgen_reset, clkgen_program;

/*
If not used elsewhere, /LRESET, /NMI and /LRESETNMIEN should be pulled high.

Work in 100us ticks (~ 500 UFM clock cycles @ 3.3-5.5MHz).

Global board enable starts 10ms after CPLD power-on.

De-bounce all power good signals over 400us.

Power-up:
1. /POR, /RESET, /RESETFULL low
2. Enable CVDD
3. Wait for CVDD good
4. Enable PLL
5. Enable CVDD1
6. Wait for CVDD1 good
7. Wait for PLL
8. Enable DVDD18
9. Wait for DVDD18 good
10. Enable DVDD15
11. Wait for DVDD15 good
12. /RESET high
13. Wait 200us
14. /POR high
15. Set boot configuration, enable DSP bank
15. Wait 100us
16. /RESETFULL high
17. Wait for /RESETSTAT to go high

Power-down:
1. /POR, /RESET, /RESETFULL go low
2. Disable DSP bank
3. Disable DVDD15
4. Wait for DVDD15 not good
5. Disable DVDD18
6. Wait for DVDD18 not good
7. Disable CVDD1
8. Wait for CVDD1 not good
9. Disable PLL
10. Disable CVDD
11. Wait for CVDD not good

If any power input is not good, run power-down sequence immediately.
*/

reg[3:0] next_state;
reg[7:0] state_timer;
reg[12:0] delay_timer;
wire delay = (delay_timer == 13'b1111111111111); /* 3msec delay */
wire timeout = (state_timer == 8'b11111111); /* 768msec timeout */

parameter off = 4'b0000,
          /* Start-up sequence */
          startup_awaiting_cvdd_good = 4'b0001,
          startup_awaiting_cvdd1_good = 4'b0010,
          startup_awaiting_dvdd18_good = 4'b0011,
          startup_awaiting_pll_locked_dvdd15_good = 4'b0100,
          startup_reset_wait_state = 4'b0101,
          startup_reset_wait_state_2 = 4'b0110,
          startup_por_wait_state = 4'b0111,
          startup_awaiting_resetstat_INV = 4'b1000,
          on = 4'b1001,
          /* Shut-down sequence */
          shutdown_reset = 4'b1010,
          shutdown_awaiting_dvdd15_off = 4'b1011,
          shutdown_awaiting_dvdd18_off = 4'b1100,
          shutdown_awaiting_cvdd1_off = 4'b1101,
          shutdown_awaiting_cvdd_pll_off = 4'b1110,
          /* Should never hit this */
          invalid1 = 4'b1111;

/*
Program the CDCE62002 shortly after it starts up, overriding whatever's in
flash.
*/
cdce62002 clkgen_interface(
   .clk(sysclk),
   .reset(clkgen_reset),
   .active(clkgen_busy),
   .send_data(clkgen_program),
   .spi_clk(pll_spi_clk),
   .spi_le(pll_spi_cs_INV),
   .spi_mosi(pll_spi_mosi),
   .spi_miso()
);

/*
"bootmode" is actually {pciessmode[1:0], bootmode[12:0], lendian} on the DSP
datasheet. Always boot in little-endian (bit 0 high), with PCIESSMODE = 00
(endpoint mode).

For bootmode_none, there is no additional configuration necessary -- just wait
for attachment via the debug port.

For bootmode_spiflash, the SPI configuration bitfieldsa re
*/
parameter boot_endian_little = 1'b1,
          boot_device_none = 3'b0,
          boot_device_spiflash = 3'b110,
          boot_config_none = 10'b0,
          boot_config_spi_mode = 2'b10, /* CPOL=0, CPHA=0 */
          boot_config_spi_pins_4 = 1'b0,
          boot_config_spi_pins_5 = 1'b1,
          boot_config_spi_addrwidth_24 = 1'b1,
          /*
          Why yes, a CS value of 2'b10 does mean CS0 enabled:
          http://e2e.ti.com/support/dsp/c6000_multi-core_dsps/f/639/t/283454.aspx
          */
          boot_config_spi_chipselect = 2'b10,
          boot_config_spi_param_table_idx = 4'b0;

wire[15:0] default_bootmode = {boot_config_none, boot_device_none,
                         boot_endian_little};
wire[15:0] spi_bootmode = {boot_config_spi_mode, boot_config_spi_pins_4,
                     boot_config_spi_addrwidth_24,
                     boot_config_spi_chipselect,
                     boot_config_spi_param_table_idx,
                     boot_device_spiflash, boot_endian_little};

/* Mapping between current state and outputs */
always @(*) begin
    /* Defaults for unhandled case statements -- everything off */
    cvdd_en = 1'b0;
    cvdd1_en = 1'b0;
    dvdd18_en = 1'b0;
    dvdd15_en = 1'b0;
    pll_en = 1'b0;
    por_INV = 1'b0;
    reset_INV = 1'b0;
    resetfull_INV = 1'b0;
    vid_oe_INV = 1'b1;
    bootmode_en = 1'b1;
    bootmode = 16'b0;
    dsp_bank_en = 1'b0;
    clkgen_reset = 1'b0;
    clkgen_program = 1'b0;

    case (state)
        off: begin
            /* Default state */
            clkgen_reset = 1'b1;
        end
        startup_awaiting_cvdd_good: begin
            cvdd_en = 1'b1;
        end
        startup_awaiting_cvdd1_good: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            pll_en = 1'b1;
        end
        startup_awaiting_dvdd18_good: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            pll_en = 1'b1;
            clkgen_program = 1'b1;
        end
        startup_awaiting_pll_locked_dvdd15_good: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            vid_oe_INV = 1'b0;
        end
        startup_reset_wait_state: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            reset_INV = 1'b1;
            vid_oe_INV = 1'b0;
            dsp_bank_en = 1'b1;
        end
        startup_reset_wait_state_2: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            reset_INV = 1'b1;
            vid_oe_INV = 1'b0;
            dsp_bank_en = 1'b1;
        end
        startup_por_wait_state: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            por_INV = 1'b1;
            reset_INV = 1'b1;
            vid_oe_INV = 1'b0;
            dsp_bank_en = 1'b1;
            bootmode = spi_bootmode;
        end
        startup_awaiting_resetstat_INV: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            por_INV = 1'b1;
            reset_INV = 1'b1;
            resetfull_INV = 1'b1;
            vid_oe_INV = 1'b0;
            dsp_bank_en = 1'b1;
            bootmode = spi_bootmode;
        end
        on: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
            por_INV = 1'b1;
            reset_INV = 1'b1;
            resetfull_INV = 1'b1;
            vid_oe_INV = 1'b0;
            dsp_bank_en = 1'b1;
            bootmode_en = 1'b0; /* Boot mode pins can be used for GPIO now */
        end
        shutdown_reset: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            dvdd15_en = 1'b1;
            pll_en = 1'b1;
        end
        shutdown_awaiting_dvdd15_off: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            dvdd18_en = 1'b1;
            pll_en = 1'b1;
        end
        shutdown_awaiting_dvdd18_off: begin
            cvdd_en = 1'b1;
            cvdd1_en = 1'b1;
            pll_en = 1'b1;
        end
        shutdown_awaiting_cvdd1_off: begin
            cvdd_en = 1'b1;
            pll_en = 1'b1;
        end
        shutdown_awaiting_cvdd_pll_off: begin
            /* Everything off -- same as "off" state */
        end
		  invalid1: begin
				/* Same as off state */
		  end
    endcase
end

/* Decide state transitions */
always @(*) begin
    next_state = state;
    case (state)
        off: begin
            /*
            Look for enable & timeout so that there's a minimum 25ms delay
            between shutting down and starting back up.
            */
            if (enable & timeout) begin
                next_state = startup_awaiting_cvdd_good;
            end
        end
        startup_awaiting_cvdd_good: begin
            if (!enable | timeout) begin
                next_state = shutdown_awaiting_cvdd_pll_off;
            end else if (cvdd_ok) begin
                next_state = startup_awaiting_cvdd1_good;
            end
        end
        startup_awaiting_cvdd1_good: begin
            if (!enable | timeout | !cvdd_ok) begin
                next_state = shutdown_awaiting_cvdd_pll_off;
            end else if (cvdd1_ok) begin
                next_state = startup_awaiting_dvdd18_good;
            end
        end
        startup_awaiting_dvdd18_good: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok) begin
                next_state = shutdown_awaiting_cvdd1_off;
            end else if (dvdd18_ok) begin
                next_state = startup_awaiting_pll_locked_dvdd15_good;
            end
        end
        startup_awaiting_pll_locked_dvdd15_good: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok | !dvdd18_ok) begin
                next_state = shutdown_awaiting_dvdd18_off;
            end else if (dvdd15_ok & pll_ok) begin
                next_state = startup_reset_wait_state;
            end
        end
        startup_reset_wait_state: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok | !pll_ok |
                    !dvdd18_ok | !dvdd15_ok) begin
                next_state = shutdown_awaiting_dvdd15_off;
            end else begin
                next_state = startup_reset_wait_state_2;
            end
        end
        startup_reset_wait_state_2: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok | !pll_ok |
                    !dvdd18_ok | !dvdd15_ok) begin
                next_state = shutdown_awaiting_dvdd15_off;
            end else begin
                next_state = startup_por_wait_state;
            end
        end
        startup_por_wait_state: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok | !pll_ok |
                    !dvdd18_ok | !dvdd15_ok) begin
                next_state = shutdown_reset;
            end else begin
                next_state = startup_awaiting_resetstat_INV;
            end
        end
        startup_awaiting_resetstat_INV: begin
            if (!enable | timeout | !cvdd_ok | !cvdd1_ok | !pll_ok |
                    !dvdd18_ok | !dvdd15_ok) begin
                next_state = shutdown_reset;
            end else if (!resetstat_INV) begin
                next_state = on;
            end
        end
        on: begin
            if (!enable | !cvdd_ok | !cvdd1_ok | !pll_ok | !dvdd18_ok |
                    !dvdd15_ok) begin
                next_state = shutdown_reset;
            end
        end
        shutdown_reset: begin
            next_state = shutdown_awaiting_dvdd15_off;
        end
        shutdown_awaiting_dvdd15_off: begin
            if (timeout | !dvdd15_ok) begin
                next_state = shutdown_awaiting_dvdd18_off;
            end
        end
        shutdown_awaiting_dvdd18_off: begin
            if (timeout | !dvdd18_ok) begin
                next_state = shutdown_awaiting_cvdd1_off;
            end
        end
        shutdown_awaiting_cvdd1_off: begin
            if (timeout | !cvdd1_ok) begin
                next_state = shutdown_awaiting_cvdd_pll_off;
            end
        end
        shutdown_awaiting_cvdd_pll_off: begin
            if (timeout | !cvdd_ok) begin
                next_state = off;
            end
        end
        invalid1: begin
            next_state = off;
        end
    endcase
end

always @(posedge sysclk) begin
    delay_timer <= delay_timer + 13'b0000000000001;
end

always @(posedge delay) begin
    if (state != next_state) begin
        state <= next_state;
        state_timer <= 4'b0000;
    end else begin
        state_timer <= state_timer + 4'b0001;
    end

    /*
    De-bounce external inputs -- wait for four good samples (100us apart)
    before considering a
    */
    if (cvdd_good) begin
        if (!cvdd_ok) begin
            cvdd_good_debounce <= cvdd_good_debounce + 2'b01;
        end
    end else begin
        cvdd_good_debounce <= 2'b00;
    end

    if (cvdd1_good) begin
        if (!cvdd1_ok) begin
            cvdd1_good_debounce <= cvdd1_good_debounce + 2'b01;
        end
    end else begin
        cvdd1_good_debounce <= 2'b00;
    end

    if (dvdd18_good) begin
        if (!dvdd18_ok) begin
            dvdd18_good_debounce <= dvdd18_good_debounce + 2'b01;
        end
    end else begin
        dvdd18_good_debounce <= 2'b00;
    end

    if (dvdd15_good) begin
        if (!dvdd15_ok) begin
            dvdd15_good_debounce <= dvdd15_good_debounce + 2'b01;
        end
    end else begin
        dvdd15_good_debounce <= 2'b00;
    end

/*
Disable PLL lock check because with the current parameters, it doesn't.

However, stability is still good enough for the DSP to function as expected.

    if (pll_locked) begin
        if (!pll_ok) begin
            pll_locked_debounce <= pll_locked_debounce + 4'b001;
        end
    end else begin
        pll_locked_debounce <= 4'b000;
    end
*/
    pll_locked_debounce <= 4'b1111;

    if (resetstat_INV) begin
        if (!resetstat_ok) begin
            resetstat_INV_debounce <= resetstat_INV_debounce + 2'b01;
        end
    end else begin
        resetstat_INV_debounce <= 2'b00;
    end
end

endmodule
