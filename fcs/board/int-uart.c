/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <assert.h>

#include <c6x.h>

#include "../c66x-csl/ti/csl/cslr.h"
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_uart.h"
#include "../c66x-csl/ti/csl/cslr_tpcc.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"

#include "board.h"
#include "int-uart.h"

/*
EDMA3 configuration

RX buffer transfers are A-synchronised, i.e. one transfer request [TR] per
event, which is triggered on every byte received. Two PaRAM entries are used
per UART: a primary entry and a reload entry. The primary entry is linked to
the reload entry, such that on each completion of the primary entry the reload
entry's destination address and transfer count is copied across. The reload
entry is linked-to-self to avoid the need for further updates.

For the RX PaRAM entries, the base destination address is the address of byte
0 in the RX buffer, and the transfer byte count (BCNT) is the size of the
buffer.

TX buffer transfers are also A-synchronised, but because the number of bytes
to be transferred varies, and only one write request at a time is permitted,
it's somewhat simpler -- just set up a single PaRAM entry per write and don't
accept further writes until it's complete.
*/

static volatile CSL_TpccRegs* edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;

/* GPIO registers */
static volatile CSL_GpioRegs* gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;

static uint16_t rx_edma_event[2] = { 4u, 14u },
                tx_edma_event[2] = { 5u, 15u };

/*
Track the last set buffer size for RX and TX transfers, so we can work out
how many bytes have been transferred by looking at BCNT.
*/
static uint16_t rx_last_buf_size[2] = { 0, 0 },
                tx_last_buf_size[2] = { 0, 0 };

/*
UART registers (table 3-1 in SPRUGP1):

RBR: Receive Buffer Register (read-only)
THR: Transmitter Holding Register (write-only)
IER: Interrupt Enable Register
IIR: Interrupt Identification Register
FCR: FIFO Control Register (write-only)
LCR: Line Control Register
MCR: Modem Control Register
LSR: Line Status Register
MSR: Modem Status Register
SCR: Scratch Pad Register
DLL: Divisor LSB Latch
DLH: Divisor MSB Latch
REVID1: Revision Identification Register 1
REVID2: Revision Identification Register 2
PWREMU_MGMT: Power and Emulation Management Register
MDR: Mode Definition Register

If using the shared addresses, RBR and THR can only be accessed when the DLAB
bit in LCR is low; DLL can only be accessed when the DLAB bit in LCR is high.

IER can only be accessed when DLAB is low. DLH can only be accessed when DLAB
is high.

HOWEVER, the C66 CSL uses *dedicated* addresses for DLH and DLL, so we don't
need to mess around with that -- they can be written at any time.
*/

static volatile CSL_UartRegs *uart[2] = {
    (CSL_UartRegs*)CSL_UART_REGS,
    (CSL_UartRegs*)CSL_UART_B_REGS
};

static uint32_t uart_baud[2] = { 115200u, 115200u };

void fcs_int_uart_reset(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /* Initialization process as described in part 2.7 of SPRUGP1 */

    /*
    PWREMU_MGMT: Power and Emulation Management Register (section 3.13 in
    SPRUGP1)

    Bit   Field          Value         Description
    14    UTRST                        UART transmitter reset.
                                       0 = transmitter in reset state
                                       1 = transmitter enabled
    13    URRST                        UART receiver reset.
                                       0 = receiver in reset state
                                       1 = receiver enabled
    0     FREE                         Free running enable for emulation.
                                       0 = halt after current transmission if
                                           emulation even received
                                       1 = keep running regardless

    Reset the UART RX/TX, and ignore emulation -- write 1u.
    */
    uart[uart_idx]->PWREMU_MGMT = 1u;

    /*
    Deactivate any outstanding EDMA3 transfers and reset the channel
    controllers etc. Since all transfers are A-synchronised, a maximum of one
    byte can be in-flight at any time.

    Disable the RX and TX DMA by setting the clear bit in the appropriate
    register. For channels 0-31, this is EECR; for channels 32+, it's EECRH.

    We also want to clear out secondary events and event missed registers, so
    that errors in transfers don't block future transfers. These registers are
    SECR/SECRH and EMCR/EMCRH respectively.

    Finally, clear out any pending events by setting the clear bits in
    ECR/ECRH.
    */
    edma3->TPCC_EECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(rx_edma_event[uart_idx],
                               rx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event[uart_idx],
                               tx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;

    /*
    The UART baud rate generator is derived from SYSCLK7 via PLLOUT->PLLDIV7.
    SYSCLK7 is always 1/6th the rate of SYSCLK1.

    Set the baud rate divisor:
        DLH:DLL = SYSCLK7_FREQ_HZ / (UART_BAUD * 13)

    so with a 100MHz clock and e.g. a desired baud rate of 921600 we'd set
        DLH:DLL = 166666667 / (921600 * 13) = 13.9

    which would result in an actual baud rate of
        ACTUAL_UART_BAUD = SYSCLK7_FREQ_HZ / (DLH:DLL * 14)
        915750 = 166666667 / (14 * 13)

    for an error of 0.63%.
    */
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    float divisor = 166666666.67f / (float)(uart_baud[uart_idx] * 13);
    uint16_t divisor_int = (uint16_t)(divisor + 0.5f);
    uart[uart_idx]->DLL = divisor_int & 0xFFu;
    uart[uart_idx]->DLH = (divisor_int >> 8) & 0xFFu;

    /*
    MDR: Mode Definition Register (section 3.14 in SPRUGP1)

    Bit   Field          Value         Description
    0     OSM_SEL                      0 = 16x oversampling
                                       1 = 13x oversampling

    Set to 0 for 13x oversampling (better for 230400, 921600 etc).
    */
    uart[uart_idx]->MDR = 0x01u;

    /*
    FCR: FIFO Control Register (section 3.5 in SPRUGP1)

    Bit   Field          Value         Description
    7:6   RXFIFTL        0-3           Receiver FIFO interrupt trigger level.
                                       0 = 1 byte
                                       1 = 4 bytes
                                       2 = 8 bytes
                                       3 = 14 bytes
    3     DMAMODE1                     Must be 1 for EDMA to work.
                                       0 = DMA mode disabled
                                       1 = DMA mode enabled
    2     TXCLR                        Clears transmitter FIFO.
                                       0 = no change
                                       1 = clear transmitter FIFO
    1     RXCLR                        Clears receiver FIFO.
                                       0 = no change
                                       1 = clear receiver FIFO
    0     FIFOEN                       Enables the RX/TX FIFOs. Must be set
                                       *before* other bits are written, or
                                       they will be ignored.
                                       0 = FIFO disabled
                                       1 = FIFO enabled

    First, set FCR to 0x01 to enable.
    Then, set 0x0F for 1-byte FIFO trigger level, RX/TX FIFO clear, and FIFO
    enable.
    Then, set 0x09 for 1-byte FIFO trigger level and FIFO enable.
    */
    uart[uart_idx]->FCR = 0x01u;
    uart[uart_idx]->FCR = 0x0Fu;
    uart[uart_idx]->FCR = 0x09u;

    /*
    LCR: Line Control Register (section 3.6 in SPRUGP1)

    Bit   Field          Value         Description
    7     DLAB                         Divisor latch access bit.
                                       0 = acccess to THR/RBR and IER,
                                           *no* access to DLL and DLH on
                                           shared address
                                       1 = access to DLL and DLH, *no* access
                                           to THR/RBR and IER on shared
                                           address
    6     BC                           Break control.
                                       0 = break condition disabled
                                       1 = break condition transmitted to
                                           receiving UART
    5     SP                           Stick parity. If PEN=1, EPS=0 and SP=1,
                                       the parity bit is active high. If
                                       PEN=1, EPS=1 and SP=1, the parity bit
                                       is active low. No effect if PEN=0.
                                       0 = stick parity disabled
                                       1 = stick parity enabled
    4     EPS                          Even parity select. Ignored if PEN=0.
                                       0 = odd parity (odd number of 1s)
                                       1 = even parity (even number of 1s)
    3     PEN                          Parity enable.
                                       0 = no parity transmitted or checked
                                       1 = parity generated and checked
    2     STB                          Number of stop bit generated.
                                       0 = one stop bit generated
                                       1 = 1.5 stop bits generated when WLS=0,
                                           2 stop bits generated when WLS>0
    1:0   WLS            0-3           Word length select.
                                       0 = 5 bits
                                       1 = 6 bits
                                       2 = 7 bits
                                       3 = 8 bits

    Since we're using dedicated addresses for DLL/DLH, we don't need to set
    DLAB.

    Set LCR to 0x03 to run with no parity, one stop bit, and 8 data bits.
    */
    uart[uart_idx]->LCR = 0x03u;

    /*
    MCR: Modem Control Register (section 3.7 in SPRUGP1)

    Bit   Field          Value         Description
    31:6  Reserved
    5     AFE                          Auto flow control enable.
                                       0 = disabled
                                       1 = enabled
    4     LOOP                         Loopback mode. Connects RX to TX
                                       internally. UARTn_TXD pin is high,
                                       UARTn_RXD is disconnected.
                                       0 = disabled
                                       1 = enabled (looped)
    3     OUT2                         OUT2 control bit.
    2     OUT1                         OUT1 control bit.
    1     RTS                          RTS control. When AFE = 1, RTS
                                       determines whether UARTn_RTS is
                                       enabled.
                                       0 = disabled
                                       1 = enabled
    0     Reserved

    Not used -- set to 0, or 0x10u for loop mode if debugging.
    */
    uart[uart_idx]->MCR = 0x10u;

    /*
    IER: Interrupt Enable Register (section 3.3 in SPRUGP1)

    Bit   Field          Value         Description
    3     EDSSI          0             Enable modem status interrupt.
    2     ELSI                         Reeiver line status interrupt enable.
                                       0 = disabled
                                       1 = enabled
    1     ETBEI                        THR empty interrupt enable.
                                       0 = disabled
                                       1 = enabled
    0     ERBI                         Receiver data available/character
                                       timeout interrupt enable.
                                       0 = disabled
                                       1 = enabled

    We don't want any interrupts, so set to 0.
    */
    uart[uart_idx]->IER = 0;

    uint8_t dummy = uart[uart_idx]->LSR;
    #pragma unused(dummy);
}

void fcs_int_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {
    assert(uart_idx == 0 || uart_idx == 1);
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    uart_baud[uart_idx] = baud;
}

uint32_t fcs_int_uart_check_error(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    uint8_t lsr = uart[uart_idx]->LSR;

    /*
    LSR: Line Status Register (section 3.8 in SPRUGP1)

    Bit   Field          Value         Description
    31:8  Reserved
    7     RXFIFOE                      RX FIFO error
                                       0 = no errors
                                       1 = at least one parity error, framing
                                           error or break indicator in the
                                           FIFO
    6     TEMT                         TX empty
                                       0 = TX FIFO or TSR contains a character
                                       1 = Neither TX FIFO nor TSR contains a
                                           character
    5     THRE                         THR empty
                                       0 = TX FIFO contains a character
                                       1 = TX FIFO empty
    4     BI                           Break indicator. Set when the RX input
                                       is held low for more than a full word
                                       time.
                                       0 = Next RX FIFO byte is not a break
                                       1 = Next RX FIFO byte is a break
    3     FE                           Framing error in RX.
                                       0 = Next RX FIFO byte does not have a
                                           framing error
                                       1 = Next RX FIFO byte has a framing
                                           error
    2     PE                           Parity error in RX.
                                       0 = Next RX FIFO byte does not have a
                                           parity error
                                       1 = Next RX FIFO byte has a parity
                                           error
    1     OE                           Overrun error in RX FIFO.
                                       0 = No overrun error
                                       1 = Overrun error
    0     DR                           Data ready in RX.
                                       0 = No byte in RBR
                                       1 = Byte in RBR
    */

    /* Error if RXFIFOE, BI, FE or OE was set */
    return lsr & 0x9Au;
}

/*
EDMA event/channel mapping: on the C6657, the mapping between events and
channels is fixed. The ID numbers are therefore interchangeable.

Events relevant to us (per SPRS814A) are, in general:

Event/Channel Number    Event      Description
2                       TINT2L     Timer2 interrupt low
3                       TINT2H     Timer2 interrupt high
4                       URXEVT     UART0 receive event
5                       UTXEVT     UART0 transmit event
6                       GPINT0     GPIO interrupt
7                       GPINT1     GPIO interrupt
8                       GPINT2     GPIO interrupt
9                       GPINT3     GPIO interrupt
14                      URXEVT_B   UART1 receive event
15                      UTXEVT_B   UART1 transmit event
16                      SPIINT0    SPI interrupt
17                      SPIINT1    SPI interrupt
22                      TINT4L     Timer4 interrupt low
23                      TINT4H     Timer4 interrupt high
24                      TINT5L     Timer5 interrupt low
25                      TINT5H     Timer5 interrupt high
26                      TINT6L     Timer6 interrupt low
27                      TINT6H     Timer6 interrupt high
28                      TINT7L     Timer7 interrupt low
29                      TINT7H     Timer7 interrupt high
30                      SPIXEVT    SPI transmit event
31                      SPIREVT    SPI receive event

In this particular driver (for internal UARTs), we only use URXEVT, UTXEVT,
URXEVT_B, and UTXEVT_B.

The UART peripheral is connected to EDMA3 transfer controllers TC0 and TC2 via
TeraNet bridges 1 (controller side) and 4 (perhiperhal side).

The EMIF16 peripheral is connected to EDMA3 transfer controllers TC0, TC1, TC2
and TC3 (i.e. all of them).

The C6657 has 512 PaRAM set entries, 64 channels, and 4 event queues. There is
a 1:1 mapping between queues and transfer controllers, i.e. Q0 -> TC0,
Q1 -> TC1 etc.

Because the 64 DMA channels map to 64 pre-assigned events (see above), and
section 2.3 of SPRUGS5A states that the first N PaRAM sets are mapped to the
DMA channels (where N is the number of DAM channels), we should ensure that
the primary PaRAM sets for each transfer type are stored in the index
corresponding to the event/channel number.

According to the same section of SPRUGS5A, "by default, all channels map to
PaRAM set 0" -- this seems to contradict the preceding but it can't hurt to
attempt to map the channels we need to the values they should already be,
right?

Addressing the PaRAM sets may be tricky since the datasheet talks about global
addresses rather than core-local addresses. We may need to convert input
addresses into whatever the "global" format is.
*/

void fcs_int_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Track this so we can return how many bytes have been written to the RX
    buffer by the EDMA engine
    */
    rx_last_buf_size[uart_idx] = buf_size;

    /*
    For an internal UART, we configure it in the mode required for DMA,
    set up a single DMA transfer per buffer with ACNT=1, BCNT=256,
    CCNT=1.

    A linked PaRAM set is configured to reload the transfer with exactly the
    same settings.

    The next destination byte index from the primary PaRAM set can be read
    directly to get the count of bytes transferred; if it's lower than the
    read pointer, add 256. (And subtract 256 once the read pointer reaches
    256.)

    First, we need to make sure the queue, event, channel and primary PaRAM
    numbers are mapped. The primary PaRAM number has to be the same as the
    DMA channel number, which in turn has to be the same as the event number,
    but apparently (per SPRUGS5A) they're unmapped by default, so do that now.

    The queue setting determines which transfer controller (TC) we use. For
    the UART, either TC0 or TC2 must be used, therefore the queue must be 0 or
    2.

    Start by disabling the channel.
    */

    /* Disable the UART RX */
    uart[uart_idx]->PWREMU_MGMT &= 0xFFFFDFFFu;
    uint8_t dummy = uart[uart_idx]->LSR;
    #pragma unused(dummy);

    /*
    Disable the channel by setting the clear bit in the appropriate register.
    For channels 0-31, this is EECR; for channels 32+, it's EECRH.
    */
    edma3->TPCC_EECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);

    /*
    8 channels per register; determine the DMAQNUM register index based
    on the channel number = event number.
    */
    uint8_t dma_register_idx = rx_edma_event[uart_idx] >> 3;

    /*
    Determine the bits of the register to set -- 4 bits per channel with the
    3 LSBs being the queue number, and the MSB being reserved.

    We don't actually need to change the setting for this particular driver as
    we're using Q0, but it's included for the sake of completeness.
    */
    uint8_t dma_register_bit = (rx_edma_event[uart_idx] -
                                (dma_register_idx << 3)) << 2;

    /* Update the DMAQNUMn register register */
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 0);

    /*
    Now we need to map the PaRAM set to the channel. The DCHMAPn registers
    have bits 31:14 reserved, 13:5 as PAENTRY, and 4:0 reserved. PAENTRY is
    the index of the PaRAM set to map, from 0-1FFh.
    */
    edma3->TPCC_DCHMAP[rx_edma_event[uart_idx]] =
        rx_edma_event[uart_idx] << 5u;

    /*
    Configure the PaRAM sets (SPRUGS5A table 2-2)

    Byte Offset     Field           Description
    0h              OPT             Tranfer configuration options.
    4h              SRC             Source address (byte address from which
                                    data is transferred).
    8h              ACNT(15:0)      Count for 1st (A) dimension; 1-65535.
                    BCNT(31:16)     Count for 2nd (B) dimension; 1-65535.
    Ch              DST             Destination address (byte address to which
                                    data is transferred).
    10h             SRCBIDX(15:0)   Source BCNT index: signed offset between
                                    source arrays (B dimension), -32768 to
                                    32767 bytes.
                    DSTBIDX(31:16)  Destination BCNT index: signed offset
                                    between destination arrays (B dimension),
                                    -32768 to 32767 bytes.
    14h             LINK(15:0)      The address of the PaRAM set to be linked
                                    when this one is complete (values in this
                                    PaRAM set are reloaded from the linked
                                    PaRAM set). 0xFFFFu is a null link (no
                                    reload). For normal PaRAM sets, the 5 LSBs
                                    should be 0 since they're 32-byte aligned.
                    BCNTRLD(31:16)  The value BCNT should be set to when BCNT
                                    is 0 and CCNT is decremented. Only used
                                    for A-synchronised transfers.
    18h             SRCCIDX(15:0)   Source CCNT index: signed offset between
                                    frames (B dimension) within a block (C
                                    dimension). For A-synchronised transfers,
                                    this is the offset from the beginning of
                                    the LAST source array (A dimension) in a
                                    frame (B dimension) to the beginning of
                                    the FIRST source array (A dimension) in
                                    the next frame (B dimension). For
                                    AB-synchronised transfers, it's the
                                    offset between the beginning of the FIRST
                                    source array (A dimension) in a frame
                                    (B dimension) to the beginning of the
                                    FIRST source array (A dimension) in the
                                    next frame (B dimension). -32768 to 32767
                                    bytes.
                    DSTCIDX(31:16)  Same as above, but for destination arrays
                                    and frames.
    1Ch             CCNT(15:0)      Number of frames (B dimension) in a block
                                    (C dimension). 1-65535.
                    RSVD(31:16)     Reserved

    Then, the OPT field (offset 0h):

    Bit   Field          Value         Description
    31    PRIV (R)                     Privilege level for the PaRAM set.
                                       Automatically set based on EDMA3 master
                                       privilege value.
                                       0 = user level
                                       1 = supervisor level
    30:28 Reserved                     Must always write 0 to these bits.
    27:24 PRIVID (R)     0-Fh          Privilege identification. Set with the
                                       EDMA3 master's privilege level when
                                       PaRAM set written.
    23    ITCCHEN                      Intermediate transfer completion
                                       chaining enable. When enabled, CER/CERH
                                       is set on completion of every
                                       intermediate transfer reqeust (TR) --
                                       but not the final TR. The bit set in
                                       CER/CERH is the TCC value specified.
                                       0 = disabled
                                       1 = enabled
    22    TCCHEN                       Same as the above, but for the final
                                       TR.
    21    ITCINTEN                     Intermediate transfer completion
                                       interrupt enable. When enabled, the
                                       IPR/IPRH bit identified by TCC is set
                                       on completion of every intermediate TR
                                       (not the final). If the corresponding
                                       IER/IERH bit is set, an interrupt is
                                       generated.
                                       0 = disabled
                                       1 = enabled
    20    TCINTEN                      Same as above, but for the final TC.
    19:18 Reserved
    17:12 TCC            0-3Fh         Transfer complete code. Identifies the
                                       bit set in CER/CERH and IPR/IPRH when
                                       the intermediate or final TRs are
                                       complete (see above).
    11    TCCMODE                      Transfer complete mode:
                                       0 = transfer is complete when the data
                                           is transferred
                                       1 = transfer is complete after the CC
                                           sends the TR to the TC.
    10:8  FWID           0-5h          FIFO width:
                                       0 = 8-bit
                                       1 = 16-bit
                                       2 = 32-bit
                                       3 = 64-bit
                                       4 = 128-bit
                                       5 = 256-bit
    7:4   Reserved                     Must always write 0 to these bits.
    3     STATIC                       If enabled, the PaRAM set is not
                                       updated on TR submission or completion.
                                       0 = disabled
                                       1 = enabled
    2     SYNCDIM                      Transfer synchronisation dimension.
                                       0 = A-synchronised (one TR of ACNT
                                           bytes per event)
                                       1 = AB-synchronised (BCNT TRs of ACNT
                                           bytes per event)
    1     DAM                          Destination address mode.
                                       0 = INCR (increment) mode. Within an
                                           array (A dimension), the
                                           destination address is incremented.
                                       1 = CONST (constant) destination
                                           address. Wraps around when FIFO
                                           width is reached.
    0     SAM                          Same as above, but for source address.

    For UART RX, we're not using completion codes or interrupts, so those
    fields can be zeroed out. The FIFO width is 8 bits, so that can be 0 as
    well. We don't want STATIC mode, because the fields need to be reloaded
    from the secondary PaRAM set. Per the UART and EDMA3 device docs (SPRUGP1
    and SPRUGS5A), we need SYNCDIM to be 0 for A-synchronised, and DAM/SAM
    should both be 0 for INCR mode. Thus, the option word is 0.

    The source address should be the RBR of the relevant UART; ACNT should be
    1 (since we're transferring one byte per event) and BCNT should be the
    buffer size. The destination address should be that of the first byte of
    the read buffer.

    The source BIDX should be 0, since we want to read bytes from the same
    address (the UART's RBR) with each transfer. The destination BIDX should
    be 1, since we want to write the bytes to the read buffer sequentially.

    The LINK field should be the PaRAM set address offset for the secondary
    (linked) PaRAM set, i.e. index * 32. The BCNTRLD field is 0, because we
    don't reload BCNT.

    The source and dest CIDXs are 0, because we don't use CCNT; CCNT itself
    is 1.

    For the reload PaRAM, we want everything to be the same, including the
    lined PaRAM index, since we're doing a self-reload.
    */
    #define primary (edma3->PARAMSET[rx_edma_event[uart_idx]])
    primary.OPT = 0;
    primary.SRC = (uint32_t)&(uart[uart_idx]->RBR);
    primary.A_B_CNT = (buf_size << 16) | 1u;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.DST = GLOBAL_FROM_L2_ADDRESS(buf);
    primary.SRC_DST_BIDX = 1u << 16;
    primary.LINK_BCNTRLD = (100u + rx_edma_event[uart_idx]) << 5;
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    #define reload (edma3->PARAMSET[100u + rx_edma_event[uart_idx]])
    reload.OPT = 0;
    reload.SRC = (uint32_t)&(uart[uart_idx]->RBR);
    reload.A_B_CNT = (buf_size << 16) | 1u;
    /* FIXME: make sure buf is actually a L2 local address */
    reload.DST = GLOBAL_FROM_L2_ADDRESS(buf);
    reload.SRC_DST_BIDX = 1u << 16;
    reload.LINK_BCNTRLD = (100u + rx_edma_event[uart_idx]) << 5;
    reload.SRC_DST_CIDX = 0;
    reload.CCNT = 1u;
    #undef reload

    /*
    First, reset DMA event status for this channel, and clear all missed
    events.

    Enable the channel by setting the enable bit in the appropriate register.
    For channels 0-31, this is EESR; for channels 32+, it's EESRH.
    */
    edma3->TPCC_SECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(rx_edma_event[uart_idx],
                               rx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;
    edma3->TPCC_EESR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);

    /* Enable the UART RX */
    uart[uart_idx]->PWREMU_MGMT |= 0x2000u;
}

void fcs_int_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    /*
    For TX in internal UARTs, we DMA-trigger an A-synchronised transfer with
    ACNT = 1, BCNT = number of bytes to send, and CCNT = 0xFFFF (don't ask).
    If there's a transfer currently in progress, we fail (see assert below).

    See comments in fcs_int_uart_start_rx_edma for register details; only the
    differences in configuration are mentioned below.
    */

    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Track buffer size so we can return number of bytes read from TX buffer
    based on the current BCNT value
    */
    tx_last_buf_size[uart_idx] = buf_size;

    /* Put the UART TX into reset -- clear UTRST in PWREMU_MGMT (bit 14) */
    uart[uart_idx]->PWREMU_MGMT &= 0xFFFFBFFF;
    uint8_t dummy = uart[uart_idx]->LSR;
    #pragma unused(dummy);

    /* Disable the DMA event for this channel */
    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);

    /* Set up DMA channel -> queue mapping */
    uint8_t dma_register_idx = tx_edma_event[uart_idx] >> 3;
    uint8_t dma_register_bit = (tx_edma_event[uart_idx] -
                                (dma_register_idx << 3)) << 2;
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 0);

    /* Map PaRAM set to channel */
    edma3->TPCC_DCHMAP[tx_edma_event[uart_idx]] =
        tx_edma_event[uart_idx] << 5u;

    #define primary (edma3->PARAMSET[tx_edma_event[uart_idx]])
    primary.OPT = 0;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.SRC = GLOBAL_FROM_L2_ADDRESS(buf); /* Read from buf */
    primary.A_B_CNT = (buf_size << 16) | 1u;
    primary.DST = (uint32_t)&(uart[uart_idx]->THR); /* Write to THR */
    primary.SRC_DST_BIDX = 1u; /* Increment src address, not dest */
    primary.LINK_BCNTRLD = 0xFFFFu; /* NULL PaRAM set for link */
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    /* Reset DMA event status for the channel and enable the transfer */
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event[uart_idx],
                               tx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;
    edma3->TPCC_EESR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);

    /* Take the UART TX out of reset to send the first event */
    uart[uart_idx]->PWREMU_MGMT |= 0x4000u;
}

uint16_t fcs_int_uart_get_rx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    uint16_t nbytes = 0;
    if (rx_last_buf_size[uart_idx] != 0) {
        /*
        Subtract BCNT from last buffer size to get the number of bytes written
        to RX buffer
        */
        nbytes = rx_last_buf_size[uart_idx] -
               (edma3->PARAMSET[rx_edma_event[uart_idx]].A_B_CNT >> 16u);
    }

    /* If there are bytes available, turn the output LED on */
    if (nbytes > 0) {
        gpio->BANK_REGISTERS[0].OUT_DATA |= 0x00800000 << (uart_idx ? 4u : 0);
    } else {
        gpio->BANK_REGISTERS[0].OUT_DATA &= 0xFF7FFFFF << (uart_idx ? 4u : 0);
    }

    return nbytes;
}

uint16_t fcs_int_uart_get_tx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    uint16_t nbytes = 0;
    if (tx_last_buf_size[uart_idx] != 0) {
        /*
        Subtract BCNT from last buffer size to get the number of bytes read
        from TX buffer
        */
        nbytes = tx_last_buf_size[uart_idx] -
               (edma3->PARAMSET[tx_edma_event[uart_idx]].A_B_CNT >> 16);
    }

    /* If there are bytes available, turn the output LED on */
    if (nbytes > 0) {
        gpio->BANK_REGISTERS[0].OUT_DATA |= 0x00400000 << (uart_idx ? 4u : 0);
    } else {
        gpio->BANK_REGISTERS[0].OUT_DATA &= 0xFFBFFFFF << (uart_idx ? 4u : 0);
    }

    return nbytes;
}
