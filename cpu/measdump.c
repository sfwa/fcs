#include <errno.h>
#include <asm/termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>

#define RD_BUFSIZE 256u
#define WR_BUFSIZE 256u

int set_interface_attribs(int fd, int speed) {
    struct termios2 tty;
    memset(&tty, 0, sizeof tty);

    ioctl(fd, TCGETS2, &tty);
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | ICRNL | IMAXBEL | BRKINT);         // ignore break signal
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 255u;
    tty.c_cc[VTIME] = 0u;

    ioctl(fd, TCSETS2, &tty);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: measdump /PATH/TO/DIR");
        return 1;
    }

    /* I/O board 1 */
    char *fname1 = tempnam(argv[1], "meas1-");
    FILE *f1;
    f1 = fopen(fname, "wb");

    int ifd1 = open("/dev/ttySAC2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ifd1 < 0) {
        printf("error %d opening /dev/ttySAC2: %s", errno, strerror(errno));
        return 1;
    }

    set_interface_attribs(ifd1, 2604168);

    /* I/O board 2 */
    char *fname2 = tempnam(argv[1], "meas2-");
    FILE *f2;
    f2 = fopen(fname2, "wb");

    int ifd2 = open("/dev/ttySAC3", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ifd2 < 0) {
        printf("error %d opening /dev/ttySAC3: %s", errno, strerror(errno));
        return 1;
    }

    set_interface_attribs(ifd2, 2604168);

    /* DSP */
    char *fname3 = tempnam(argv[1], "meas3-");
    FILE *f3;
    f3 = fopen(fname3, "wb");

    int ifd3 = open("/dev/ttySAC0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ifd3 < 0) {
        printf("error %d opening /dev/ttySAC0: %s", errno, strerror(errno));
        return 1;
    }

    set_interface_attribs(ifd3, 921600);

    uint64_t n_written = 0;
    char *buf = malloc(RD_BUFSIZE);

    while (1) {
        int n = read(ifd1, buf, RD_BUFSIZE);
        if (n > 0) {
            fwrite(buf, 1, n, f1);
            n_written += n;
        }
        n = read(ifd2, buf, RD_BUFSIZE);
        if (n > 0) {
            fwrite(buf, 1, n, f2);
            n_written += n;
        }
        n = read(ifd3, buf, RD_BUFSIZE);
        if (n > 0) {
            fwrite(buf, 1, n, f3);
            n_written += n;
        }
        if (n_written > 65535) {
            fflush(f1);
            fflush(f2);
            fflush(f3);
        }
    }
}
