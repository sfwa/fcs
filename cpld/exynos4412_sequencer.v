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


module exynos4412_sequencer(
    input sysclk,
    input enable,
    output reg cpu_pmic_pwron,
    output reg cpu_pmic_reset_INV,
    output reg cpu_bank_en,
    output reg[5:0] cpu_bootmode
);

/*
Work in 100us ticks (~ 500 UFM clock cycles @ 3.3-5.5MHz).

Global board enable starts 10ms after CPLD power-on.

Power-up:
1. cpu_pmic_pwron and cpu_pmic_reset_INV low
2. cpu_bootmode tri-state/open-drain
3. cpu_pmic_pwron high
4. Wait 10ms
5. Set cpu_bootmode (6'b101001 for eMMC, 6'b000101 for MicroSD)
6. Start up CPU bank
7. Wait 10ms
8. cpu_pmic_reset_INV high

Power-down:
1. cpu_pmic_reset_INV low
2. Remove boot mode config, shut down CPU bank
2. Wait 10ms
3. cpu_pmic_pwron low
*/

reg[2:0] state;
reg[2:0] next_state;
reg[6:0] state_timer;
wire timeout_10ms = (state_timer == 7'b1100100);

parameter off = 3'b000,
          /* Start-up sequence */
          startup_pwron_wait_state = 3'b001,
          startup_bootmode_wait_state = 3'b010,
          on = 3'b0011,
          /* Shut-down sequence */
          shutdown_reset_wait_state = 3'b100,
          /* Should never hit these */
          invalid1 = 3'b101,
          invalid2 = 3'b110,
          invalid3 = 3'b111;

/* Mapping between current state and outputs */
always @(*) begin
    /* Defaults for unhandled case statements -- everything off */
    cpu_pmic_pwron = 1'b0;
    cpu_pmic_reset_INV = 1'b0;
    cpu_bootmode = 6'b000000;
    cpu_bank_en = 1'b0;

    case (state)
        off: begin
            /* Default state */
        end
        startup_pwron_wait_state: begin
            cpu_pmic_pwron = 1'b1;
        end
        startup_bootmode_wait_state: begin
            cpu_pmic_pwron = 1'b1;
            cpu_bootmode = 6'b000101; /* MicroSD boot */
            cpu_bank_en = 1'b1;
        end
        on: begin
            cpu_pmic_pwron = 1'b1;
            cpu_pmic_reset_INV = 1'b1;
            cpu_bootmode = 6'b000101; /* MicroSD boot */
            cpu_bank_en = 1'b1;
        end
        shutdown_reset_wait_state: begin
            cpu_pmic_pwron = 1'b1;
        end
		  invalid1: begin
				/* Same as off state */
		  end
		  invalid2: begin
				/* Same as off state */
		  end
		  invalid3: begin
				/* Same as off state */
		  end
    endcase
end

/* Decide state transitions */
always @(*) begin
    next_state = state;
    case (state)
        off: begin
            /* Default state */
        end
        startup_pwron_wait_state: begin
            /*
            Look for enable & timeout so that there's a minimum 10ms delay
            between shutting down and starting back up.
            */
            if (enable & timeout_10ms) begin
                next_state = startup_pwron_wait_state;
            end
        end
        startup_bootmode_wait_state: begin
            if (!enable) begin
                next_state = shutdown_reset_wait_state;
            end else if (timeout_10ms) begin
                next_state = on;
            end
        end
        on: begin
            if (!enable) begin
                next_state = shutdown_reset_wait_state;
            end
        end
        shutdown_reset_wait_state: begin
            if (timeout_10ms) begin
                next_state = off;
            end
        end
        invalid1: begin
            next_state = off;
        end
        invalid2: begin
            next_state = off;
        end
        invalid3: begin
            next_state = off;
        end
    endcase
end

always @(posedge sysclk) begin
    if (state != next_state) begin
        state <= next_state;
        state_timer <= 7'b00000000;
    end else begin
        state_timer <= state_timer + 7'b00000001;
    end
end

endmodule
