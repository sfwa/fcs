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

/* Implements a PMBus pass-through device, without clock stretching support */
module pmbus_passthrough(
    input reset,
    input sysclk,
    inout master_scl,
    inout master_sda,
    output slave_scl,
    inout slave_sda,
    output sda_direction_tap
);

parameter MOSI = 1'b0,
          MISO = 1'b1;
parameter IDLE = 3'b000,
          ADDRESS = 3'b001,
          RW = 3'b010,
          SLAVE_ACK = 3'b011,
          MASTER_ACK = 3'b100,
          DATA_TO_SLAVE = 3'b101,
          DATA_FROM_SLAVE = 3'b110;

/*
Clock pass-through from master (out) to slave (in) -- no clock stretching
support since that would require tri-stating slave_scl to check whether it's
still being held low.

If this is an issue, this module could handle the clock stretching itself by
setting master_scl as an open-drain inout and holding it low for a preset
period after each negative edge.
*/
assign slave_scl = master_scl == 1'b0 ? 1'b0 : 1'bz;
assign master_scl = 1'bz;

reg[3:0] state;
reg[4:0] scl_samples;
reg[4:0] sda_samples;
reg scl_new, scl_old, sda_new, sda_old, got_start, got_stop, master_sda_bit,
    slave_sda_bit, sda_direction;

assign slave_sda = (sda_direction == MOSI) ? master_sda : 1'bz;
assign master_sda = (sda_direction == MISO) ? slave_sda : 1'bz;
assign sda_direction_tap = (sda_direction == MISO) ? 1'b1 : 1'b0;

/* Read the bus, and de-bounce SCL and SDA */
always @(posedge sysclk or posedge reset) begin
    if (reset) begin
        scl_samples <= 5'b11111;  /* Bus is pulled up when inactive */
        sda_samples <= 5'b11111;
        scl_new <= 1'b1;
        scl_old <= 1'b1;
        sda_new <= 1'b1;
        sda_old <= 1'b1;
        got_start <= 1'b0;
        got_stop <= 1'b0;
    end else begin
        /* Sample SCL/SDA and store them in the shift register */
        scl_samples <= {scl_samples[3:0], master_scl};
        sda_samples <= {sda_samples[3:0], master_sda};

        /* Save previous SCL, and check if there's a clear new value */
        scl_old <= scl_new;
        if (scl_samples == 5'b11111) begin
            scl_new <= 1;
        end else if (scl_samples == 5'b00000) begin
            scl_new <= 0;
        end

        /* Do the same for SDA */
        sda_old <= sda_new;
        if (sda_samples == 5'b11111) begin
            sda_new <= 1;
        end else if (sda_samples == 5'b00000) begin
            sda_new <= 0;
        end

        /*
        If SCL remained high on the falling edge of SDA, it's a start
        condition.

        The start condition is cleared on the falling edge of SDA.
        */
        if (scl_new & scl_old & !sda_new & sda_old) begin
            got_start <= 1;
        end else if (!scl_new & !scl_old) begin
            got_start <= 0;
        end

        /*
        If SCL remained high on the rising edge of SDA, it's a stop condition.

        The condition is cleared on the falling edge of SCL.
        */
        got_stop <= scl_new & scl_old & sda_new & !sda_old;
    end
end

/* Latch the data bits on the positive edge of each clock cycle */
always @(posedge master_scl) begin
    master_sda_bit <= master_sda;
    slave_sda_bit <= slave_sda;
end

/*
Decode enough of the PMBus protocol to work out which side is meant to be
driving SDA
*/
reg [3:0] bitcount; /* 8 bits of data + ACK */
reg isread;         /* R/W direction */
reg newcycle;

always @(negedge master_scl or posedge reset or posedge got_start or posedge got_stop) begin
    if (reset | got_start | got_stop) begin
        state <= IDLE;
        sda_direction <= MOSI;
        bitcount <= 4'd7;
        isread <= 0;
    end else begin
        case (state)
            IDLE: begin
                state <= ADDRESS;
                /* Start at bit 6 because the first edge is missed */
                bitcount <= 4'd6;
            end
            ADDRESS: begin
                if (bitcount == 4'd1) begin
                    state <= RW; /* Address done, so next bit is R/W */
                end else begin
                    bitcount <= bitcount - 4'd1;
                end
            end
            RW: begin
                isread <= master_sda_bit;
                sda_direction <= MISO;
                state <= SLAVE_ACK;
            end
            SLAVE_ACK: begin
                /* Wait for a byte of data */
                bitcount <= 4'd7;
                if (isread) begin
                    sda_direction <= MISO;
                    state <= DATA_FROM_SLAVE;
                end else begin
                    sda_direction <= MOSI;
                    state <= DATA_TO_SLAVE;
                end
            end
            DATA_FROM_SLAVE: begin
                if (bitcount == 4'd0) begin
                    /* Finished reading a byte from the slave */
                    sda_direction <= MOSI;
                    state <= MASTER_ACK;
                end else begin
                    bitcount <= bitcount - 4'd1;
                end
            end
            MASTER_ACK: begin
                /*
                Either start/stop or start on the next byte. If start/stop,
                go back to the beginning
                */
                if (master_sda_bit == 1) begin
                    /* NACK from slave; send a STOP next */
                    sda_direction <= MOSI;
                    state <= IDLE;
                end else begin
                    /* Waiting for a full byte */
                    bitcount <= 4'd7;
                    sda_direction <= MISO;
                    state <= DATA_FROM_SLAVE;
                end
            end
            DATA_TO_SLAVE: begin
                if (bitcount == 4'h0) begin
                    /* Just finished a byte */
                    sda_direction <= MISO;
                    state <= SLAVE_ACK;
                end else begin
                    bitcount <= bitcount - 4'd1;
                end
            end
        endcase
    end
end

endmodule
