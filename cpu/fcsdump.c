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

static FILE *outputFile;
static int exitRequested = 0;

static void sigintHandler(int signum) {
    exitRequested = 1;
}

static int readCallback(uint8_t *buffer, int length,
FTDIProgressInfo *progress, void *userdata) {
    if (length) {
        if (fwrite(buffer, length, 1, outputFile) != 1) {
            perror("Write error");
            return 1;
        }
   }
   if (progress) {
       fprintf(stderr, "%10.02fs total time %9.3f MiB captured %7.1f kB/s curr rate %7.1f kB/s totalrate %d dropouts\n",
               progress->totalTime,
               progress->current.totalBytes / (1024.0 * 1024.0),
               progress->currentRate / 1024.0,
               progress->totalRate / 1024.0);
   }
   return exitRequested ? 1 : 0;
}

int main(int argc, char **argv) {
    struct ftdi_context *ftdi;
    int err, c;
    FILE *of = NULL;
    char const *outfile  = 0;
    outputFile =0;
    exitRequested = 0;
    char *descstring = NULL;

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

    if (ftdi_usb_open_desc(ftdi, 0x0403, 0x6014, descstring, NULL) < 0) {
        fprintf(stderr, "Can't open ftdi device: %s\n",
                ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    /* A timeout value of 1 results in may skipped blocks */
    if (ftdi_set_latency_timer(ftdi, 2)) {
        fprintf(stderr, "Can't set latency, Error %s\n",
               ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    char *fname = tempnam(argv[1], "fcs-");
    of = fopen(fname, "wb");

    setvbuf(of, NULL, _IOFBF , 1 << 16);
    outputFile = of;
    signal(SIGINT, sigintHandler);

    err = ftdi_readstream(ftdi, readCallback, NULL, 8, 256);
    if (err < 0 && !exitRequested) {
        exit(1);
    }

    fclose(outputFile);
    outputFile = NULL;

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
