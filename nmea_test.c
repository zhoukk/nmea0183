#define NMEA_0183_IMPLEMENTATION
#include "nmea_0183.h"

#define UART_IMPLEMENTATION
#include "uart.h"

#define READLINE_IMPLEMENTATION
#include "readline.h"

int
main(int argc, char *argv[]) {
    int fd;

    if (argc < 2) {
        printf("usage: %s devpath\n", argv[0]);
        return 0;
    }

    fd = uart_open(argv[1], 9600, 0, 8, 'N', 1);
    if (fd < 0) {
        fprintf(stderr, "fatal: uart_open(): %s: %s\n", argv[1], strerror(errno));
        exit(-1);
    }

    readline_t rl = {0};
    for (;;) {
        char *sentence = 0;
        char buffer[256] = {0};
        ssize_t len = read(fd, buffer, 256);

        readline_push(&rl, buffer, len);
        while ((sentence = (readline_pop(&rl)))) {
            nmea_0183_t p;

            nmea_0183_parse(sentence, &p);
            nmea_0183_print(&p);

            char buf[128] = {0};
            nmea_0183_serialize(&p, buf);

            printf("s: %s\n", sentence);
            printf("d: %s\n", buf);
        }
    }

    uart_close(fd);

    // char sentence[] = "$GNGGA,203415.000,6325.6138,N,01021.4290,E,1,8,2.42,72.5,M,41.5,M,,*7C\r\n";
    // char sentence[] = "$GPRMC,064216.00,A,3116.11411,N,12044.55665,E,0.212,,290622,,,A*7C\r\n";
    // nmea_0183_t p;
    // nmea_0183_parse(sentence, &p);
    // nmea_0183_print(&p);

    // char buf[128] = {0};
    // nmea_0183_serialize(&p, buf);

    // printf("%s\n", buf);

    return 0;
}
