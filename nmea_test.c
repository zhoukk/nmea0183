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

            printf("s: %s\n", sentence);

            int ret = nmea_0183_parse(sentence, &p);
            if (ret != 0) {
                printf("ret:%d\n", ret);
            }

            if (p.type == NMEA_SENTENCE_RMC) {
                double longitude;
                double latitude;
                nmea_wgs84_bd09(p.rmc.longitude, p.rmc.latitude, &longitude, &latitude);
                printf("gps: [%.10f,%.10f]\n", longitude, latitude);
            }

            char buf[128] = {0};
            nmea_0183_serialize(&p, buf);

            printf("d: %s\n", buf);

            free(sentence);
        }
    }

    uart_close(fd);

    return 0;
}
