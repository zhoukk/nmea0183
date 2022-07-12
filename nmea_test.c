#define NMEA_0183_IMPLEMENTATION
#include "nmea_0183.h"

#define UART_IMPLEMENTATION
#include "uart.h"

#define READLINE_IMPLEMENTATION
#include "readline.h"

#define KALMAN_FILTER_IMPLEMENTATION
#include "kalman_filter.h"

static kalman_filter_t *
gps_filter_init() {
    kalman_filter_t *kf = kalman_filter_new(4, 2);

    matrix_identity(kf->state_transition);

    kf->state_transition.data[0][2] = 0.001;
    kf->state_transition.data[1][3] = 0.001;

    matrix_set(kf->observation_model, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    double pos = 0.000001;
    matrix_set(kf->process_noise_covariance, pos, 0.0, 0.0, 0.0, 0.0, pos, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               1.0);

    double noise = 1.0;

    matrix_set(kf->observation_noise_covariance, pos * noise, 0.0, 0.0, pos * noise);

    matrix_set(kf->state_estimate, 0.0, 0.0, 0.0, 0.0);
    matrix_identity(kf->estimate_covariance);
    const double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
    matrix_scale(kf->estimate_covariance, trillion);

    return kf;
}

static void
gps_filter_update(kalman_filter_t *kf, double in_longitude, double in_latitude, double *out_longitude,
                  double *out_latitude) {
    // if (first == 1) {
    //     double init_longitude[2] = {in_longitude, 0};
    //     double init_longitude_p[2][2] = {{0, 0}, {0, 0}};
    //     kalman_filter_2d_init(&longitude_filter, init_longitude, init_longitude_p);

    //     double init_latitude[2] = {in_latitude, 0};
    //     double init_latitude_p[2][2] = {{0, 0}, {0, 0}};
    //     kalman_filter_2d_init(&latitude_filter, init_latitude, init_latitude_p);
    //     first = 0;
    // }

    // double process_noise_err[2] = {0.00001, 0.00001};
    // double measurement_err = (in_longitude - longitude_filter.x[0]) * (in_longitude - longitude_filter.x[0]);
    // measurement_err += (in_latitude - latitude_filter.x[0]) * (in_latitude - latitude_filter.x[0]);
    // measurement_err *= 1300;

    // longitude_filter.q[0] = process_noise_err[0];
    // longitude_filter.q[1] = process_noise_err[1];
    // longitude_filter.r = measurement_err;

    // latitude_filter.q[0] = process_noise_err[0];
    // latitude_filter.q[1] = process_noise_err[1];
    // latitude_filter.r = measurement_err;

    // *out_longitude = kalman_filter_2d(&longitude_filter, in_longitude);
    // *out_latitude = kalman_filter_2d(&latitude_filter, in_latitude);

    matrix_set(kf->observation, in_longitude, in_latitude);

    kalman_filter_update(kf);

    *out_longitude = kf->state_estimate.data[0][0];
    *out_latitude = kf->state_estimate.data[1][0];
}

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

    kalman_filter_t *kf = gps_filter_init();

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

            if (p.type == NMEA_SENTENCE_RMC && p.rmc.validity == NMEA_STATUS_VALID) {
                double longitude;
                double latitude;
                nmea_wgs84_gcj02(p.rmc.longitude, p.rmc.latitude, &longitude, &latitude);

                double opt_longitude;
                double opt_latitude;
                gps_filter_update(kf, longitude, latitude, &opt_longitude, &opt_latitude);
                printf("%.10f,%.10f - %.10f,%.10f\n", longitude, latitude, opt_longitude, opt_latitude);
            }

            // char buf[128] = {0};
            // nmea_0183_serialize(&p, buf);

            // printf("d: %s\n", buf);

            free(sentence);
        }
    }

    uart_close(fd);

    return 0;
}
