#include "test.h"

#include <cstdlib>

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "drivers/stream.h"

/* Prototypes for private test hooks */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t maxlen);
}

TEST(StreamRX, BasicRead) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(8, len);

    /* Read it back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 256);
    EXPECT_EQ(8, len);
    EXPECT_STREQ((char*)s, (char*)buf);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(0, len);
}

TEST(StreamRX, PartialReads) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(8, len);

    /* Read the first part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    buf[4] = 0;
    EXPECT_EQ(4, len);
    EXPECT_STREQ("test", (char*)buf);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(4, len);

    /* Read the second part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    EXPECT_STREQ("ing", (char*)buf);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(0, len);
}

TEST(StreamRX, MixedReadsWrites) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read the first part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    buf[4] = 0;

    EXPECT_STREQ("test", (char*)buf);

    /* Write another test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read the remainder back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    EXPECT_STREQ("ing", (char*)buf);

    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 256);
    EXPECT_EQ(8, len);
    EXPECT_STREQ("testing", (char*)buf);
}

TEST(StreamRX, MixedReadsPeeks) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    ASSERT_EQ(8, len);

    EXPECT_EQ((int16_t)'t', fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_EQ((int16_t)'t', fcs_stream_peek(FCS_STREAM_UART_INT0));

    /* Read the first part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    ASSERT_EQ(4, len);

    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));

    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 256);
    EXPECT_EQ(4, len);
    EXPECT_STREQ("ing", (char*)buf);

    EXPECT_EQ((int16_t)-1, fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_EQ((int16_t)-1, fcs_stream_peek(FCS_STREAM_UART_INT0));
}

TEST(StreamRX, SkipUntilAfter) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(8, len);

    /* Skip the first 't' */
    len = fcs_stream_skip_until_after(FCS_STREAM_UART_INT0, 't');
    EXPECT_EQ(1, len);
    EXPECT_EQ((int16_t)'e', fcs_stream_peek(FCS_STREAM_UART_INT0));

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(7, len);

    /* Now the second */
    len = fcs_stream_skip_until_after(FCS_STREAM_UART_INT0, 't');
    EXPECT_EQ(3, len);
    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(4, len);

    /* Now we should get 0 returned */
    len = fcs_stream_skip_until_after(FCS_STREAM_UART_INT0, 't');
    EXPECT_EQ(0, len);

    /* But the current read pointer shouldn't change */
    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));
}

TEST(StreamRX, ReadUntilAfter) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Skip the first 't' */
    len = fcs_stream_read_until_after(FCS_STREAM_UART_INT0, 't', buf, 256);
    buf[1] = 0;
    EXPECT_EQ(1, len);
    EXPECT_EQ((int16_t)'e', fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_STREQ("t", (char*)buf);

    /* Now the second */
    len = fcs_stream_read_until_after(FCS_STREAM_UART_INT0, 't', buf, 256);
    buf[3] = 0;
    EXPECT_EQ(3, len);
    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_STREQ("est", (char*)buf);

    /* Now we should get 0 returned */
    len = fcs_stream_read_until_after(FCS_STREAM_UART_INT0, 't', buf, 256);
    EXPECT_EQ(0, len);

    /* But the current read pointer shouldn't change */
    EXPECT_EQ((int16_t)'i', fcs_stream_peek(FCS_STREAM_UART_INT0));

    /* Now read the rest */
    len = fcs_stream_read_until_after(FCS_STREAM_UART_INT0, 'g', buf, 256);
    buf[3] = 0;
    EXPECT_EQ(3, len);
    EXPECT_EQ((int16_t)0, fcs_stream_peek(FCS_STREAM_UART_INT0));
    EXPECT_STREQ("ing", (char*)buf);
}
