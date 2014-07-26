#include <errno.h>
#include <asm/termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <ftdi.h>

static int exitRequested = 0;

static void sigintHandler(int signum) {
    exitRequested = 1;
}

int main(int argc, char **argv) {
    struct ftdi_context *ftdi;
    unsigned char buf[32768];
    exitRequested = 0;
    FILE *of = NULL;

    if (argc < 2) {
        printf("Usage: fcsdump /PATH/TO/DIR");
        return 1;
    }

    if ((ftdi = ftdi_new()) == 0) {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    if (ftdi_set_interface(ftdi, INTERFACE_A) < 0) {
        fprintf(stderr, "ftdi_set_interface failed\n");
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    if (ftdi_usb_open(ftdi, 0x0403, 0x6014) < 0) {
        fprintf(stderr, "Can't open ftdi device: %s\n",
                ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    char *fname = tempnam(argv[1], "fcs-");
    of = fopen(fname, "wb");
    setvbuf(of, NULL, _IOFBF , 1 << 16);
    signal(SIGINT, sigintHandler);

    while (!exitRequested)
    {
        int n = ftdi_read_data(ftdi, buf, sizeof(buf));
        if (n <= 0) {
            usleep(100);
        } else {
            fwrite(buf, sizeof(char), n, of);
        }
    }

    fclose(of);

    fprintf(stderr, "Capture ended.\n");

    if (ftdi_set_bitmode(ftdi,  0xff, BITMODE_RESET) < 0) {
        fprintf(stderr, "Can't set RESET mode, Error %s\n",
                ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }
    ftdi_usb_close(ftdi);
    ftdi_free(ftdi);
    signal(SIGINT, SIG_DFL);
    exit (0);
}
