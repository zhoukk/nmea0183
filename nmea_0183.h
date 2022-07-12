/*
 * nmea_0183.h -- nmea 0183 defines, structures and utils.
 *
 * https://en.wikipedia.org/wiki/NMEA_0183
 *
 * Copyright (c) zhoukk <izhoukk@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _NMEA_0183_H_
#define _NMEA_0183_H_

/* generic includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_ARGS_MAX 30

/* NMEA Talker types */
typedef enum {
    NMEA_TALKER_UNKNOWN,
    NMEA_TALKER_GP,
    NMEA_TALKER_GL,
    NMEA_TALKER_GA,
    NMEA_TALKER_BD,
    NMEA_TALKER_GN,
} nmea_talker_type_t;

/* NMEA sentence types */
typedef enum {
    NMEA_SENTENCE_UNKNOWN,
    NMEA_SENTENCE_GGA,
    NMEA_SENTENCE_GSA,
    NMEA_SENTENCE_GSV,
    NMEA_SENTENCE_RMC,
    NMEA_SENTENCE_VTG,
    NMEA_SENTENCE_GLL,
    NMEA_SENTENCE_TXT,
    NMEA_SENTENCE_ZDA,
} nmea_sentence_type_t;

typedef enum {
    NMEA_DIR_NORTH = 'N',
    NMEA_DIR_SOUTH = 'S',
    NMEA_DIR_EAST = 'E',
    NMEA_DIR_WEST = 'W',
} nmea_dir_t;

typedef enum {
    NMEA_STATUS_VALID = 'A',
    NMEA_STATUS_INVALID = 'V',
} nmea_status_t;

typedef enum {
    NMEA_FAA_UNKNOWN = 0,
    NMEA_FAA_AUTONOMOUS = 'A',
    NMEA_FAA_DIFFERENTIAL = 'D',
    NMEA_FAA_ESTIMATED = 'E',
    NMEA_FAA_NOTVALID = 'N',
    NMEA_FAA_SIMULATED = 'S',
    NMEA_FAA_MANUAL = 'M',
} nmea_faa_mode_t;

typedef struct {
    int day;
    int month;
    int year;
} nmea_date_t;

typedef struct {
    int hour;
    int minute;
    int second;
    int microsecond;
} nmea_time_t;

/*
 ** GGA - Global Positioning System Fix Data
 ** Time, Position and fix related data fora GPS receiver.
 **
 **                                                      11
 **        1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
 **        |         |       | |        | | |  |   |   | |   | |   |    |
 ** $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
 **
 ** Field Number:
 **  1) Universal Time Coordinated (UTC)
 **  2) Latitude
 **  3) N or S (North or South)
 **  4) Longitude
 **  5) E or W (East or West)
 **  6) GPS Quality Indicator,
 **     0 - fix not available,
 **     1 - GPS fix,
 **     2 - Differential GPS fix
 **  7) Number of satellites in view, 00 - 12
 **  8) Horizontal Dilution of precision
 **  9) Antenna Altitude above/below mean-sea-level (geoid)
 ** 10) Units of antenna altitude, meters
 ** 11) Geoidal separation, the difference between the WGS-84 earth
 **     ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level
 **     below ellipsoid
 ** 12) Units of geoidal separation, meters
 ** 13) Age of differential GPS data, time in seconds since last SC104
 **     type 1 or 9 update, null field when DGPS is not used
 ** 14) Differential reference station ID, 0000-1023
 ** 15) Checksum
 */
typedef enum {
    NMEA_FIX_QUALITY_INVALID = '0',
    NMEA_FIX_QUALITY_GPS_FIX = '1',
    NMEA_FIX_QUALITY_DGPS_FIX = '2',
} nmea_fix_quality_t;

typedef struct {
    nmea_time_t time;
    double latitude;
    double longitude;
    nmea_fix_quality_t fix_quality;
    int num_satellites;
    double hdop;
    double altitude;
    char altitude_unit;
    double separation;
    char separation_unit;
    double dgps_age;
    int dgps_id;
} nmea_sentence_gga_t;

/*
 ** GSA - GPS DOP and Active Satellites
 **
 **        1 2 3  4  5  6  7  8  9  10 11 12 13 14 15  16  17  18
 **        | | |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |
 ** $--GSA,a,x,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,x.x,x.x,x.x*hh<CR><LF>
 **
 ** Field Number:
 **  1) Operating Mode, A = Automatic, M = Manual
 **  2) Fix Mode, 1 = Fix not available, 2 = 2D, 3 = 3D
 **  3) Satellite PRN #1
 **  4) Satellite PRN #2
 **  5) Satellite PRN #3
 **  6) Satellite PRN #4
 **  7) Satellite PRN #5
 **  8) Satellite PRN #6
 **  9) Satellite PRN #7
 ** 10) Satellite PRN #8
 ** 11) Satellite PRN #9
 ** 12) Satellite PRN #10
 ** 13) Satellite PRN #11
 ** 14) Satellite PRN #12
 ** 15) PDOP
 ** 16) HDOP
 ** 17) VDOP
 ** 18) Checksum
 */
typedef enum {
    NMEA_GSA_MODE_AUTO = 'A',
    NMEA_GSA_MODE_FORCED = 'M',
} nmea_gsa_mode_t;

typedef enum {
    NMEA_GSA_FIX_NONE = '1',
    NMEA_GSA_FIX_2D = '2',
    NMEA_GSA_FIX_3D = '3',
} nmea_gsa_fix_type_t;

typedef struct {
    nmea_gsa_mode_t mode;
    nmea_gsa_fix_type_t fix_type;
    int sv[12];
    double pdop;
    double hdop;
    double vdop;
} nmea_sentence_gsa_t;

/*
 ** GSV - TRANSIT Position - Latitude/Longitude
 ** Location and time of TRANSIT fix at waypoint
 **
 **        1 2 3  4  5  6   7  8  9  10  11 12 13 14  15 16 17 18  19  20
 **        | | |  |  |  |   |  |  |  |   |  |  |  |   |  |  |  |   |   |
 ** $--GSV,x,x,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,*hh<CR><LF>
 **
 **  1) Total number of messages, 1-3
 **  2) Message Number, 1-3
 **  3) Total number of satellites in view
 **  4) Satellite Number #1
 **  5) Elevation #1
 **  6) Azimuth, Degrees True #1
 **  7) SNR #1, NULL when not tracking
 **  8) Satellite Number #2
 **  9) Elevation #2
 ** 10) Azimuth, Degrees True #2
 ** 11) SNR #2, NULL when not tracking
 ** 12) Satellite Number #3
 ** 13) Elevation #3
 ** 14) Azimuth, Degrees True #3
 ** 15) SNR #3, NULL when not tracking
 ** 16) Satellite Number #4
 ** 17) Elevation #4
 ** 18) Azimuth, Degrees True #4
 ** 19) SNR #4, NULL when not tracking
 ** 20) Checksum
 */
typedef struct {
    int prn;
    int elevation;
    int azimuth;
    int snr;
} nmea_sat_info_t;

typedef struct {
    int total_messages;
    int message_number;
    int number_svs_inview;
    nmea_sat_info_t sats[4];
} nmea_sentence_gsv_t;

/*
 ** RMC - Recommended Minimum Navigation Information
 **                                                            12
 **        1         2 3       4 5        6 7   8   9    10  11|
 **        |         | |       | |        | |   |   |    |   | |
 ** $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh<CR><LF>
 **
 ** Field Number:
 **  1) UTC Time
 **  2) Status, V = Navigation receiver warning
 **  3) Latitude
 **  4) N or S
 **  5) Longitude
 **  6) E or W
 **  7) Speed over ground, knots
 **  8) Track made good, degrees true
 **  9) Date, ddmmyy
 ** 10) Magnetic Variation, degrees
 ** 11) E or W
 ** 12) FAA Mode (version 2.3)
 ** 13) Checksum
 */
typedef struct {
    nmea_time_t time;
    nmea_status_t validity;
    double latitude;
    double longitude;
    double speed;
    double course;
    nmea_date_t date;
    double variation;
    nmea_faa_mode_t faa_mode;
} nmea_sentence_rmc_t;

/*
 ** VTG - Track made good and Ground speed
 **
 **        1   2 3   4 5	 6 7   8 9
 **        |   | |   | |	 | |   | |
 ** $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
 **
 ** Field Number:
 **  1) Track Degrees
 **  2) T = True
 **  3) Track Degrees
 **  4) M = Magnetic
 **  5) Speed Knots
 **  6) N = Knots
 **  7) Speed Kilometers Per Hour
 **  8) K = Kilometers Per Hour
 **  9) Checksum
 */
typedef struct {
    double true_track_degrees;
    char true_type;
    double magnetic_track_degrees;
    char magnetic_type;
    double speed_knots;
    char speed_knots_unit;
    double speed_kph;
    char speed_kph_unit;
    nmea_faa_mode_t faa_mode;
} nmea_sentence_vtg_t;

/*
 ** GLL - Geographic Position - Latitude/Longitude
 ** Latitude, N/S, Longitude, E/W, UTC, Status
 **
 **        1       2 3        4 5         6 7
 **        |       | |        | |         | |
 ** $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
 **
 ** Field Number:
 **  1) Latitude
 **  2) N or S (North or South)
 **  3) Longitude
 **  4) E or W (East or West)
 **  5) Universal Time Coordinated (UTC)
 **  6) Status A - Data Valid, V - Data Invalid
 **  7) Checksum
 */
typedef struct {
    double latitude;
    double longitude;
    nmea_time_t time;
    nmea_status_t status;
    nmea_faa_mode_t faa_mode;
} nmea_sentence_gll_t;

/*
 ** TXT - TypeTXT
 */
typedef struct {
    int total_number;
    int number;
    int id;
    char *message;
} nmea_sentence_txt_t;

/*
 ** ZDA - Time & Date
 ** UTC, day, month, year and local time zone
 **
 ** $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
 **        |         |  |  |    |  |
 **        |         |  |  |    |  +- Local zone minutes description, same sign as local hours
 **        |         |  |  |    +- Local zone description, 00 to +- 13 hours
 **        |         |  |  +- Year
 **        |         |  +- Month, 01 to 12
 **        |         +- Day, 01 to 31
 **        +- Universal Time Coordinated (UTC)
 */
typedef struct {
    nmea_time_t time;
    nmea_date_t date;
    int hour_offset;
    int minute_offset;
} nmea_sentence_zda_t;

typedef struct {
    nmea_talker_type_t talker;
    nmea_sentence_type_t type;
    union {
        nmea_sentence_gga_t gga;
        nmea_sentence_gsa_t gsa;
        nmea_sentence_gsv_t gsv;
        nmea_sentence_rmc_t rmc;
        nmea_sentence_vtg_t vtg;
        nmea_sentence_gll_t gll;
        nmea_sentence_txt_t txt;
        nmea_sentence_zda_t zda;
    };
} nmea_0183_t;

int nmea_0183_parse(char *sentence, nmea_0183_t *p);

int nmea_0183_serialize(const nmea_0183_t *p, char sentence[128]);

void nmea_bd09_gcj02(double lng, double lat, double *glng, double *glat);

void nmea_gcj02_bd09(double lng, double lat, double *blng, double *blat);

void nmea_wgs84_gcj02(double lng, double lat, double *glng, double *glat);

void nmea_gcj02_wgs84(double lat, double lng, double *wlat, double *wlng);

void nmea_bd09_wgs84(double lat, double lng, double *wlat, double *wlng);

void nmea_wgs84_bd09(double lat, double lng, double *blat, double *blng);

int nmea_in_china(double lng, double lat);

#endif // _NMEA_0183_H_

#ifdef NMEA_0183_IMPLEMENTATION

#include <ctype.h>
#include <math.h>

typedef enum {
    FSM_OK,
    FSM_ERROR,
    FSM_INVALID,
} fsm_err_t;

typedef enum {
    FSM_STATE_INIT,
    FSM_STATE_ABORT,
    FSM_STATE_COMPLETE,
    FSM_STATE_NAME_RECV,
    FSM_STATE_ARGS_RECV,
    FSM_STATE_CSUM_RECV,
    FSM_STATE_CRLF_RECV,
    FSM_STATE_MAX,
} fsm_state_type_t;

typedef enum {
    FSM_EVENT_DOLLAR,
    FSM_EVENT_ORDINARY,
    FSM_EVENT_COMMA,
    FSM_EVENT_ASTERISK,
    FSM_EVENT_CR,
    FSM_EVENT_LF,
    FSM_EVENT_NUL,
    FSM_EVENT_MAX,
} fsm_event_type_t;

typedef struct fsm fsm_t;

typedef struct fsm_state {
    fsm_err_t (*event_handler[FSM_EVENT_MAX])(fsm_t *);
    const char *data;
} fsm_state_t;

typedef struct {
    char *args[NMEA_ARGS_MAX];
    char *name;
    char *csum;
    uint8_t args_num;
    uint32_t str_len;
} nmea_token_t;

struct fsm {
    fsm_state_t state;
    nmea_token_t token;
    uint8_t wait_nul;
    uint8_t name_len;
    uint8_t csum_len;
    uint8_t arg_sn;
    uint8_t error_code;
    char *str_ptr;
    uint32_t str_offset;
};

fsm_err_t fsm_state_set(fsm_t *fsm, fsm_state_t *state);
fsm_state_t *fsm_state_get(fsm_t *fsm);

static fsm_err_t fsm_do_nothing(fsm_t *fsm);
static fsm_err_t fsm_abort(fsm_t *fsm);
static fsm_err_t fsm_init_recv_dollar(fsm_t *fsm);
static fsm_err_t fsm_name_recv_comma(fsm_t *fsm);
static fsm_err_t fsm_args_recv_comma(fsm_t *fsm);
static fsm_err_t fsm_args_recv_asterisk(fsm_t *fsm);
static fsm_err_t fsm_csum_recv_cr(fsm_t *fsm);
static fsm_err_t fsm_crlf_recv_lf(fsm_t *fsm);
static fsm_err_t fsm_crlf_recv_nul(fsm_t *fsm);

// clang-format off
static fsm_state_t state_list[FSM_STATE_MAX] = {
    /* dollar               ordinary        comma                   asterisk                CR                  LF                  NUL                 data */
    {{fsm_init_recv_dollar, fsm_abort,      fsm_abort,              fsm_abort,              fsm_abort,          fsm_abort,          fsm_abort},         "INIT"},
    {{fsm_do_nothing,       fsm_do_nothing, fsm_do_nothing,         fsm_do_nothing,         fsm_do_nothing,     fsm_do_nothing,     fsm_do_nothing},    "ABORT"},
    {{fsm_do_nothing,       fsm_do_nothing, fsm_do_nothing,         fsm_do_nothing,         fsm_do_nothing,     fsm_do_nothing,     fsm_do_nothing},    "COMPLETE"},
    {{fsm_abort,            fsm_do_nothing, fsm_name_recv_comma,    fsm_abort,              fsm_abort,          fsm_abort,          fsm_abort},         "NAME_RECV"},
    {{fsm_abort,            fsm_do_nothing, fsm_args_recv_comma,    fsm_args_recv_asterisk, fsm_abort,          fsm_abort,          fsm_abort},         "ARGS_RECV"},
    {{fsm_abort,            fsm_do_nothing, fsm_abort,              fsm_abort,              fsm_csum_recv_cr,   fsm_abort,          fsm_abort},         "CSUM_RECV"},
    {{fsm_abort,            fsm_abort,      fsm_abort,              fsm_abort,              fsm_abort,          fsm_crlf_recv_lf,   fsm_crlf_recv_nul}, "CRLF_RECV"},
};
// clang-format on

static fsm_err_t
fsm_do_nothing(fsm_t *fsm) {
    return FSM_OK;
}

static fsm_err_t
fsm_abort(fsm_t *fsm) {
    fsm->state = state_list[FSM_STATE_ABORT];
    return FSM_OK;
}

static fsm_err_t
fsm_init_recv_dollar(fsm_t *fsm) {
    fsm->token.name = &fsm->str_ptr[fsm->str_offset + 1];
    fsm->state = state_list[FSM_STATE_NAME_RECV];
    return FSM_OK;
}

static fsm_err_t
fsm_name_recv_comma(fsm_t *fsm) {
    fsm->token.args[fsm->arg_sn++] = &fsm->str_ptr[fsm->str_offset + 1];
    fsm->str_ptr[fsm->str_offset] = '\0';
    fsm->state = state_list[FSM_STATE_ARGS_RECV];
    return FSM_OK;
}

static fsm_err_t
fsm_args_recv_comma(fsm_t *fsm) {
    if (fsm->arg_sn >= NMEA_ARGS_MAX) {
        fsm->state = state_list[FSM_STATE_ABORT];
        return FSM_OK;
    }
    fsm->token.args[fsm->arg_sn++] = &fsm->str_ptr[fsm->str_offset + 1];
    fsm->str_ptr[fsm->str_offset] = '\0';
    return FSM_OK;
}

static fsm_err_t
fsm_args_recv_asterisk(fsm_t *fsm) {
    fsm->token.args_num = fsm->arg_sn;
    fsm->token.csum = &fsm->str_ptr[fsm->str_offset + 1];
    fsm->str_ptr[fsm->str_offset] = '\0';
    fsm->state = state_list[FSM_STATE_CSUM_RECV];
    return FSM_OK;
}

static fsm_err_t
fsm_csum_recv_cr(fsm_t *fsm) {
    fsm->str_ptr[fsm->str_offset] = '\0';
    fsm->state = state_list[FSM_STATE_CRLF_RECV];
    return FSM_OK;
}

static fsm_err_t
fsm_crlf_recv_lf(fsm_t *fsm) {
    if (0 == fsm->wait_nul) {
        fsm->wait_nul = 1;
    } else {
        fsm->state = state_list[FSM_STATE_ABORT];
    }
    return FSM_OK;
}

static fsm_err_t
fsm_crlf_recv_nul(fsm_t *fsm) {
    if (1 == fsm->wait_nul) {
        fsm->state = state_list[FSM_STATE_COMPLETE];
    } else {
        fsm->state = state_list[FSM_STATE_ABORT];
    }
    return FSM_OK;
}

static nmea_talker_type_t
_nmea_get_talker_type(nmea_token_t *token) {
    char *name = token->name;
    if (0 == strncmp(name, "GP", 2)) {
        return NMEA_TALKER_GP;
    } else if (0 == strncmp(name, "GL", 2)) {
        return NMEA_TALKER_GL;
    } else if (0 == strncmp(name, "GA", 2)) {
        return NMEA_TALKER_GA;
    } else if (0 == strncmp(name, "BD", 2)) {
        return NMEA_TALKER_BD;
    } else if (0 == strncmp(name, "GN", 2)) {
        return NMEA_TALKER_GN;
    }

    return NMEA_TALKER_UNKNOWN;
}

static const char *
_nmea_get_talker_name(nmea_talker_type_t type) {
    switch (type) {
    case NMEA_TALKER_GP:
        return "GP";
    case NMEA_TALKER_GL:
        return "GL";
    case NMEA_TALKER_GA:
        return "GA";
    case NMEA_TALKER_BD:
        return "BD";
    case NMEA_TALKER_GN:
        return "GN";
    default:
        return "";
    }

    return "";
}

static nmea_sentence_type_t
_nmea_get_sentence_type(nmea_token_t *token) {
    char *type = token->name + 2;
    if (0 == strcmp(type, "GGA")) {
        return NMEA_SENTENCE_GGA;
    } else if (0 == strcmp(type, "GSA")) {
        return NMEA_SENTENCE_GSA;
    } else if (0 == strcmp(type, "GSV")) {
        return NMEA_SENTENCE_GSV;
    } else if (0 == strcmp(type, "RMC")) {
        return NMEA_SENTENCE_RMC;
    } else if (0 == strcmp(type, "VTG")) {
        return NMEA_SENTENCE_VTG;
    } else if (0 == strcmp(type, "GLL")) {
        return NMEA_SENTENCE_GLL;
    } else if (0 == strcmp(type, "TXT")) {
        return NMEA_SENTENCE_TXT;
    } else if (0 == strcmp(type, "ZDA")) {
        return NMEA_SENTENCE_ZDA;
    }

    return NMEA_SENTENCE_UNKNOWN;
}

static const char *
_nmea_get_sentence_name(nmea_sentence_type_t type) {
    switch (type) {
    case NMEA_SENTENCE_GGA:
        return "GGA";
    case NMEA_SENTENCE_GSA:
        return "GSA";
    case NMEA_SENTENCE_GSV:
        return "GSV";
    case NMEA_SENTENCE_RMC:
        return "RMC";
    case NMEA_SENTENCE_VTG:
        return "VTG";
    case NMEA_SENTENCE_GLL:
        return "GLL";
    case NMEA_SENTENCE_TXT:
        return "TXT";
    case NMEA_SENTENCE_ZDA:
        return "ZDA";
    default:
        return "";
    }

    return "";
}

static int
_token_double(nmea_token_t *t, int index, double *value) {
    if (index >= t->args_num) {
        return -1;
    }

    *value = atof(t->args[index]);

    return 0;
}

static int
_token_int(nmea_token_t *t, int index, int *value) {
    if (index >= t->args_num) {
        return -1;
    }

    *value = atoi(t->args[index]);

    return 0;
}

static int
_token_string(nmea_token_t *t, int index, char **value) {
    if (index >= t->args_num) {
        return -1;
    }

    *value = t->args[index];

    return 0;
}

static int
_token_fix_quality(nmea_token_t *t, int index, nmea_fix_quality_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];
    if (v != NMEA_FIX_QUALITY_INVALID && v != NMEA_FIX_QUALITY_GPS_FIX && v != NMEA_FIX_QUALITY_DGPS_FIX) {
        return -1;
    }

    *value = (nmea_fix_quality_t)v;

    return 0;
}

static int
_token_faa_mode(nmea_token_t *t, int index, nmea_faa_mode_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];

    *value = (nmea_faa_mode_t)v;

    return 0;
}

static int
_token_gsa_mode(nmea_token_t *t, int index, nmea_gsa_mode_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];
    if (v != NMEA_GSA_MODE_AUTO && v != NMEA_GSA_MODE_FORCED) {
        return -1;
    }

    *value = (nmea_gsa_mode_t)v;

    return 0;
}

static int
_token_gsa_fix_type(nmea_token_t *t, int index, nmea_gsa_fix_type_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];
    if (v != NMEA_GSA_FIX_NONE && v != NMEA_GSA_FIX_2D && v != NMEA_GSA_FIX_3D) {
        return -1;
    }

    *value = (nmea_gsa_fix_type_t)v;

    return 0;
}

static int
_token_track_type(nmea_token_t *t, int index, char *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];

    *value = v;

    return 0;
}

static int
_token_status(nmea_token_t *t, int index, nmea_status_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];
    if (v != NMEA_STATUS_VALID && v != NMEA_STATUS_INVALID) {
        return -1;
    }

    *value = (nmea_status_t)v;

    return 0;
}

static int
_token_unit(nmea_token_t *t, int index, char *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char v = t->args[index][0];

    *value = v;

    return 0;
}

static int
_token_latitude(nmea_token_t *t, int index, double *value) {
    if (index + 1 >= t->args_num) {
        return -1;
    }

    double dv = atof(t->args[index]);

    nmea_dir_t cv = (nmea_dir_t)t->args[index + 1][0];
    if (cv != '\0' && cv != NMEA_DIR_NORTH && cv != NMEA_DIR_SOUTH) {
        return -1;
    }

    double degrees = floor(dv / 100.0);
    double minutes = dv - (degrees * 100.0);
    double v = degrees + minutes / 60.0;

    if (cv == NMEA_DIR_SOUTH) {
        v *= -1;
    }

    *value = v;

    return 0;
}

static int
_token_longitude(nmea_token_t *t, int index, double *value) {
    if (index + 1 >= t->args_num) {
        return -1;
    }

    double dv = atof(t->args[index]);

    nmea_dir_t cv = (nmea_dir_t)t->args[index + 1][0];
    if (cv != '\0' && cv != NMEA_DIR_EAST && cv != NMEA_DIR_WEST) {
        return -1;
    }

    double degrees = floor(dv / 100.0);
    double minutes = dv - (degrees * 100.0);
    double v = degrees + minutes / 60.0;

    if (cv == NMEA_DIR_WEST) {
        v *= -1;
    }

    *value = v;

    return 0;
}

static int
_token_variation(nmea_token_t *t, int index, double *value) {
    if (index + 1 >= t->args_num) {
        return -1;
    }

    double dv = atof(t->args[index]);

    nmea_dir_t cv = (nmea_dir_t)t->args[index + 1][0];
    if (cv == NMEA_DIR_WEST) {
        dv *= -1;
    }

    *value = dv;

    return 0;
}

static int
_token_time(nmea_token_t *t, int index, nmea_time_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char *arg = t->args[index];
    size_t arg_len = strlen(arg);

    if (arg_len < 6) {
        return 0;
    }

    char harg[] = {arg[0], arg[1], '\0'};
    char marg[] = {arg[2], arg[3], '\0'};
    char sarg[] = {arg[4], arg[5], '\0'};

    value->hour = atoi(harg);
    value->minute = atoi(marg);
    value->second = atoi(sarg);

    if (arg_len > 6 && arg[6] == '.') {
        int v = 0;
        int s = 1000000;
        int i = 7;
        while (isdigit((unsigned char)arg[i]) && s > 1) {
            v = (v * 10) + (arg[i] - '0');
            s /= 10;
            i++;
        }
        value->microsecond = v * s;
    }

    return 0;
}

static int
_token_date(nmea_token_t *t, int index, nmea_date_t *value) {
    if (index >= t->args_num) {
        return -1;
    }

    char *arg = t->args[index];
    size_t arg_len = strlen(arg);

    if (arg_len < 6) {
        return 0;
    }

    char darg[] = {arg[0], arg[1], '\0'};
    char marg[] = {arg[2], arg[3], '\0'};
    char yarg[] = {arg[4], arg[5], '\0'};

    value->day = atoi(darg);
    value->month = atoi(marg);
    value->year = atoi(yarg);

    return 0;
}

static int
_nmea_parse_gga(nmea_token_t *token, nmea_0183_t *p) {
    if (_token_time(token, 0, &p->gga.time)) {
        return -1;
    }

    if (_token_latitude(token, 1, &p->gga.latitude)) {
        return -1;
    }

    if (_token_longitude(token, 3, &p->gga.longitude)) {
        return -1;
    }

    if (_token_fix_quality(token, 5, &p->gga.fix_quality)) {
        return -1;
    }

    if (_token_int(token, 6, &p->gga.num_satellites)) {
        return -1;
    }

    if (_token_double(token, 7, &p->gga.hdop)) {
        return -1;
    }

    if (_token_double(token, 8, &p->gga.altitude)) {
        return -1;
    }

    if (_token_unit(token, 9, &p->gga.altitude_unit)) {
        return -1;
    }

    if (_token_double(token, 10, &p->gga.separation)) {
        return -1;
    }

    if (_token_unit(token, 11, &p->gga.separation_unit)) {
        return -1;
    }

    if (_token_double(token, 12, &p->gga.dgps_age)) {
        return -1;
    }

    if (_token_int(token, 13, &p->gga.dgps_id)) {
        return -1;
    }

    return 0;
}

static int
_nmea_parse_gsa(nmea_token_t *token, nmea_0183_t *p) {
    if (_token_gsa_mode(token, 0, &p->gsa.mode)) {
        return -1;
    }

    if (_token_gsa_fix_type(token, 1, &p->gsa.fix_type)) {
        return -1;
    }

    for (int i = 0; i < 12; i++) {
        if (_token_int(token, i + 2, &p->gsa.sv[i])) {
            return -1;
        }
    }

    if (_token_double(token, 14, &p->gsa.pdop)) {
        return -1;
    }

    if (_token_double(token, 15, &p->gsa.hdop)) {
        return -1;
    }

    if (_token_double(token, 16, &p->gsa.vdop)) {
        return -1;
    }

    return 0;
}

static int
_nmea_parse_gsv(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_int(token, 0, &p->gsv.total_messages)) {
        return -1;
    }

    if (_token_int(token, 1, &p->gsv.message_number)) {
        return -1;
    }

    if (_token_int(token, 2, &p->gsv.number_svs_inview)) {
        return -1;
    }

    for (int i = 0; i < 4; i++) {
        _token_int(token, i * 4 + 3, &p->gsv.sats[i].prn);
        _token_int(token, i * 4 + 4, &p->gsv.sats[i].elevation);
        _token_int(token, i * 4 + 5, &p->gsv.sats[i].azimuth);
        _token_int(token, i * 4 + 6, &p->gsv.sats[i].snr);
    }

    return 0;
}

static int
_nmea_parse_rmc(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_time(token, 0, &p->rmc.time)) {
        return -1;
    }

    if (_token_status(token, 1, &p->rmc.validity)) {
        return -1;
    }

    if (_token_latitude(token, 2, &p->rmc.latitude)) {
        return -1;
    }

    if (_token_longitude(token, 4, &p->rmc.longitude)) {
        return -1;
    }

    if (_token_double(token, 6, &p->rmc.speed)) {
        return -1;
    }

    if (_token_double(token, 7, &p->rmc.course)) {
        return -1;
    }

    if (_token_date(token, 8, &p->rmc.date)) {
        return -1;
    }

    if (_token_variation(token, 9, &p->rmc.variation)) {
        return -1;
    }

    _token_faa_mode(token, 11, &p->rmc.faa_mode);

    return 0;
}

static int
_nmea_parse_vtg(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_double(token, 0, &p->vtg.true_track_degrees)) {
        return -1;
    }

    if (_token_track_type(token, 1, &p->vtg.true_type)) {
        return -1;
    }

    if (_token_double(token, 2, &p->vtg.magnetic_track_degrees)) {
        return -1;
    }

    if (_token_track_type(token, 3, &p->vtg.magnetic_type)) {
        return -1;
    }

    if (_token_double(token, 4, &p->vtg.speed_knots)) {
        return -1;
    }

    if (_token_unit(token, 5, &p->vtg.speed_knots_unit)) {
        return -1;
    }

    if (_token_double(token, 6, &p->vtg.speed_kph)) {
        return -1;
    }

    if (_token_unit(token, 7, &p->vtg.speed_kph_unit)) {
        return -1;
    }

    _token_faa_mode(token, 8, &p->vtg.faa_mode);

    return 0;
}

static int
_nmea_parse_gll(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_latitude(token, 0, &p->gll.latitude)) {
        return -1;
    }

    if (_token_longitude(token, 2, &p->gll.longitude)) {
        return -1;
    }

    if (_token_time(token, 4, &p->gll.time)) {
        return -1;
    }

    if (_token_status(token, 5, &p->gll.status)) {
        return -1;
    }

    _token_faa_mode(token, 6, &p->gll.faa_mode);

    return 0;
}

static int
_nmea_parse_txt(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_int(token, 0, &p->txt.total_number)) {
        return -1;
    }

    if (_token_int(token, 1, &p->txt.number)) {
        return -1;
    }

    if (_token_int(token, 2, &p->txt.id)) {
        return -1;
    }

    if (_token_string(token, 3, &p->txt.message)) {
        return -1;
    }

    return 0;
}

static int
_nmea_parse_zda(nmea_token_t *token, nmea_0183_t *p) {

    if (_token_time(token, 0, &p->zda.time)) {
        return -1;
    }

    if (_token_int(token, 1, &p->zda.date.day)) {
        return -1;
    }

    if (_token_int(token, 2, &p->zda.date.month)) {
        return -1;
    }

    if (_token_int(token, 3, &p->zda.date.year)) {
        return -1;
    }

    if (_token_int(token, 4, &p->zda.hour_offset)) {
        return -1;
    }

    if (_token_int(token, 5, &p->zda.minute_offset)) {
        return -1;
    }

    return 0;
}

int
nmea_0183_parse(char *sentence, nmea_0183_t *p) {
    fsm_t fsm;
    char ch;
    char check = 0;
    uint8_t checksum = 0x00;

    if (!sentence || !p) {
        return -1;
    }
    memset(p, 0, sizeof(*p));
    memset(&fsm, 0, sizeof(fsm));
    fsm.str_ptr = sentence;
    fsm.state = state_list[FSM_STATE_INIT];
    ch = sentence[fsm.str_offset];
    fsm_event_type_t event = FSM_EVENT_MAX;
    while (ch) {
        switch (ch) {
        case '$':
            event = FSM_EVENT_DOLLAR;
            check = 1;
            break;
        case ',':
            event = FSM_EVENT_COMMA;
            break;
        case '*':
            event = FSM_EVENT_ASTERISK;
            check = 0;
            break;
        case '\r':
            event = FSM_EVENT_CR;
            break;
        case '\n':
            event = FSM_EVENT_LF;
            break;
        default:
            event = FSM_EVENT_ORDINARY;
            break;
        }
        fsm.state.event_handler[event](&fsm);
        if (!strcmp(state_list[FSM_STATE_ABORT].data, fsm.state.data)) {
            return -1;
        }
        if (check && (event == FSM_EVENT_COMMA || event == FSM_EVENT_ORDINARY)) {
            checksum ^= ch;
        }
        ch = sentence[++fsm.str_offset];
    }
    fsm.state.event_handler[FSM_EVENT_NUL](&fsm);
    if (strcmp(state_list[FSM_STATE_COMPLETE].data, fsm.state.data)) {
        return -1;
    }
    fsm.token.str_len = fsm.str_offset;

    uint8_t expected = (uint8_t)strtol(fsm.token.csum, 0, 16);
    if (checksum != expected) {
        return -1;
    }

    nmea_token_t *token = &fsm.token;

    memset(p, 0, sizeof *p);
    p->talker = _nmea_get_talker_type(token);
    p->type = _nmea_get_sentence_type(token);

    switch (p->type) {
    case NMEA_SENTENCE_GGA:
        return _nmea_parse_gga(token, p);
    case NMEA_SENTENCE_GSA:
        return _nmea_parse_gsa(token, p);
    case NMEA_SENTENCE_GSV:
        return _nmea_parse_gsv(token, p);
    case NMEA_SENTENCE_RMC:
        return _nmea_parse_rmc(token, p);
    case NMEA_SENTENCE_VTG:
        return _nmea_parse_vtg(token, p);
    case NMEA_SENTENCE_GLL:
        return _nmea_parse_gll(token, p);
    case NMEA_SENTENCE_TXT:
        return _nmea_parse_txt(token, p);
    case NMEA_SENTENCE_ZDA:
        return _nmea_parse_zda(token, p);
    case NMEA_SENTENCE_UNKNOWN:
    default:
        return -1;
    }

    return -1;
}

static int
_serialize_time(const nmea_time_t *time, char *sentence) {
    return sprintf(sentence, ",%02d%02d%02d.%02d", time->hour, time->minute, time->second, time->microsecond);
}

static int
_serialize_date(const nmea_date_t *date, char *sentence) {
    return sprintf(sentence, ",%02d%02d%02d", date->day, date->month, date->year);
}

static int
_serialize_latitude(double latitude, char *sentence) {
    char dir = 'N';

    if (latitude < 0) {
        dir = 'S';
        latitude *= -1;
    }

    double value = floor(latitude) * 100.0;
    double minutes = (latitude - value / 100.0) * 60.0;
    double v = value + minutes;

    return sprintf(sentence, ",%.5f,%c", v, dir);
}

static int
_serialize_longitude(double longitude, char *sentence) {
    char dir = 'E';

    if (longitude < 0) {
        dir = 'W';
        longitude *= -1;
    }

    double value = floor(longitude) * 100.0;
    double minutes = (longitude - value / 100.0) * 60.0;
    double v = value + minutes;

    return sprintf(sentence, ",%.5f,%c", v, dir);
}

static int
_serialize_variation(double variation, char *sentence) {
    char dir = 'E';

    if (variation < 0) {
        dir = 'W';
        variation *= -1;
    }
    if (variation == 0.0) {
        return sprintf(sentence, ",,");
    }
    return sprintf(sentence, ",%g,%c", variation, dir);
}

static int
_serialize_faa_mode(nmea_faa_mode_t faa_mode, char *sentence) {
    return sprintf(sentence, ",%c", (char)faa_mode);
}

static int
_serialize_status(nmea_status_t status, char *sentence) {
    return sprintf(sentence, ",%c", (char)status);
}

static int
_serialize_gsa_mode(nmea_gsa_mode_t gsa_mode, char *sentence) {
    return sprintf(sentence, ",%c", (char)gsa_mode);
}

static int
_serialize_gsa_fix_type(nmea_gsa_fix_type_t fix_type, char *sentence) {
    return sprintf(sentence, ",%c", (char)fix_type);
}

static int
_serialize_fix_quality(nmea_fix_quality_t fix_quality, char *sentence) {
    return sprintf(sentence, ",%c", (char)fix_quality);
}

static int
_serialize_track_type(char track_type, char *sentence) {
    return sprintf(sentence, ",%c", track_type);
}

static int
_serialize_double(double value, char *sentence) {
    if (value == 0.0) {
        return sprintf(sentence, ",");
    }
    return sprintf(sentence, ",%g", value);
}

static int
_serialize_int(int value, char *sentence) {
    if (value == 0) {
        return sprintf(sentence, ",");
    }
    return sprintf(sentence, ",%02d", value);
}

static int
_serialize_string(char *value, char *sentence) {
    return sprintf(sentence, ",%s", value);
}

static int
_serialize_unit(char unit, char *sentence) {
    return sprintf(sentence, ",%c", unit);
}

static int
_nmea_serialize_gga(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_time(&p->gga.time, sentence + offset);

    offset += _serialize_latitude(p->gga.latitude, sentence + offset);

    offset += _serialize_longitude(p->gga.longitude, sentence + offset);

    offset += _serialize_fix_quality(p->gga.fix_quality, sentence + offset);

    offset += _serialize_int(p->gga.num_satellites, sentence + offset);

    offset += _serialize_double(p->gga.hdop, sentence + offset);

    offset += _serialize_double(p->gga.altitude, sentence + offset);

    offset += _serialize_unit(p->gga.altitude_unit, sentence + offset);

    offset += _serialize_double(p->gga.separation, sentence + offset);

    offset += _serialize_unit(p->gga.separation_unit, sentence + offset);

    offset += _serialize_double(p->gga.dgps_age, sentence + offset);

    offset += _serialize_int(p->gga.dgps_id, sentence + offset);

    return offset;
}

static int
_nmea_serialize_gsa(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_gsa_mode(p->gsa.mode, sentence + offset);

    offset += _serialize_gsa_fix_type(p->gsa.fix_type, sentence + offset);

    for (int i = 0; i < 12; i++) {
        offset += _serialize_int(p->gsa.sv[i], sentence + offset);
    }

    offset += _serialize_double(p->gsa.pdop, sentence + offset);

    offset += _serialize_double(p->gsa.hdop, sentence + offset);

    offset += _serialize_double(p->gsa.vdop, sentence + offset);

    return offset;
}

static int
_nmea_serialize_gsv(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_int(p->gsv.total_messages, sentence + offset);

    offset += _serialize_int(p->gsv.message_number, sentence + offset);

    offset += _serialize_int(p->gsv.number_svs_inview, sentence + offset);

    for (int i = 0; i < 4; i++) {
        if (p->gsv.sats[i].prn != 0 || p->gsv.sats[i].elevation != 0 || p->gsv.sats[i].azimuth != 0 ||
            p->gsv.sats[i].snr != 0) {
            offset += _serialize_int(p->gsv.sats[i].prn, sentence + offset);

            offset += _serialize_int(p->gsv.sats[i].elevation, sentence + offset);

            offset += _serialize_int(p->gsv.sats[i].azimuth, sentence + offset);

            offset += _serialize_int(p->gsv.sats[i].snr, sentence + offset);
        }
    }

    return offset;
}

static int
_nmea_serialize_rmc(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_time(&p->rmc.time, sentence + offset);

    offset += _serialize_status(p->rmc.validity, sentence + offset);

    offset += _serialize_latitude(p->rmc.latitude, sentence + offset);

    offset += _serialize_longitude(p->rmc.longitude, sentence + offset);

    offset += _serialize_double(p->rmc.speed, sentence + offset);

    offset += _serialize_double(p->rmc.course, sentence + offset);

    offset += _serialize_date(&p->rmc.date, sentence + offset);

    offset += _serialize_variation(p->rmc.variation, sentence + offset);

    offset += _serialize_faa_mode(p->rmc.faa_mode, sentence + offset);

    return offset;
}

static int
_nmea_serialize_vtg(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_double(p->vtg.true_track_degrees, sentence + offset);

    offset += _serialize_track_type(p->vtg.true_type, sentence + offset);

    offset += _serialize_double(p->vtg.magnetic_track_degrees, sentence + offset);

    offset += _serialize_track_type(p->vtg.magnetic_type, sentence + offset);

    offset += _serialize_double(p->vtg.speed_knots, sentence + offset);

    offset += _serialize_unit(p->vtg.speed_knots_unit, sentence + offset);

    offset += _serialize_double(p->vtg.speed_kph, sentence + offset);

    offset += _serialize_unit(p->vtg.speed_kph_unit, sentence + offset);

    if (p->vtg.faa_mode != NMEA_FAA_UNKNOWN) {
        offset += _serialize_faa_mode(p->vtg.faa_mode, sentence + offset);
    }

    return offset;
}

static int
_nmea_serialize_gll(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_latitude(p->gll.latitude, sentence + offset);

    offset += _serialize_longitude(p->gll.longitude, sentence + offset);

    offset += _serialize_time(&p->gll.time, sentence + offset);

    offset += _serialize_status(p->gll.status, sentence + offset);

    if (p->gll.faa_mode != NMEA_FAA_UNKNOWN) {
        offset += _serialize_faa_mode(p->gll.faa_mode, sentence + offset);
    }

    return offset;
}

static int
_nmea_serialize_txt(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_int(p->txt.total_number, sentence + offset);

    offset += _serialize_int(p->txt.number, sentence + offset);

    offset += _serialize_int(p->txt.id, sentence + offset);

    offset += _serialize_string(p->txt.message, sentence + offset);

    return offset;
}

static int
_nmea_serialize_zda(const nmea_0183_t *p, char sentence[128]) {
    int offset = 0;

    offset += _serialize_time(&p->zda.time, sentence + offset);

    offset += _serialize_int(p->zda.date.day, sentence + offset);

    offset += _serialize_int(p->zda.date.month, sentence + offset);

    offset += _serialize_int(p->zda.date.year, sentence + offset);

    offset += _serialize_int(p->zda.hour_offset, sentence + offset);

    offset += _serialize_int(p->zda.minute_offset, sentence + offset);

    return offset;
}

int
nmea_0183_serialize(const nmea_0183_t *p, char sentence[128]) {
    const char *talker = _nmea_get_talker_name(p->talker);
    const char *name = _nmea_get_sentence_name(p->type);

    if (talker[0] == '\0' || name[0] == '\0') {
        return -1;
    }

    int n = sprintf(sentence, "$%s%s", talker, name);

    switch (p->type) {
    case NMEA_SENTENCE_GGA:
        n += _nmea_serialize_gga(p, sentence + n);
        break;
    case NMEA_SENTENCE_GSA:
        n += _nmea_serialize_gsa(p, sentence + n);
        break;
    case NMEA_SENTENCE_GSV:
        n += _nmea_serialize_gsv(p, sentence + n);
        break;
    case NMEA_SENTENCE_RMC:
        n += _nmea_serialize_rmc(p, sentence + n);
        break;
    case NMEA_SENTENCE_VTG:
        n += _nmea_serialize_vtg(p, sentence + n);
        break;
    case NMEA_SENTENCE_GLL:
        n += _nmea_serialize_gll(p, sentence + n);
        break;
    case NMEA_SENTENCE_TXT:
        n += _nmea_serialize_txt(p, sentence + n);
        break;
    case NMEA_SENTENCE_ZDA:
        n += _nmea_serialize_zda(p, sentence + n);
        break;
    case NMEA_SENTENCE_UNKNOWN:
    default:
        return -1;
    }

    uint8_t checksum = 0x00;
    for (int i = 1; i < n; i++) {
        checksum ^= sentence[i];
    }

    sprintf(sentence + n, "*%02X\r\n", checksum);

    return 0;
}

static const double PI = 3.1415926535897932384626;
static const double X_PI = 3.1415926535897932384626 * 3000.0 / 180.0;
static const double AXIS = 6378245.0;
static const double OFFSET = 0.00669342162296594323;

static void
_transform(double longitude, double latitude, double *out_longitude, double *out_latitude) {
    double longlat = longitude * latitude;
    double abs_x = sqrt(fabs(longitude));
    double longitude_pi = longitude * PI;
    double latitude_pi = latitude * PI;
    double d = 20.0 * sin(6.0 * longitude_pi) + 20.0 * sin(2.0 * longitude_pi);
    double x = d + 20.0 * sin(latitude_pi) + 40.0 * sin(latitude_pi / 3.0);
    double y = d + 20.0 * sin(longitude_pi) + 40.0 * sin(longitude_pi / 3.0);

    x += 160.0 * sin(latitude_pi / 12.0) + 320.0 * sin(latitude_pi / 30.0);
    y += 150.0 * sin(longitude_pi / 12.0) + 300.0 * sin(longitude_pi / 30.0);
    x *= 2.0 / 3.0;
    y *= 2.0 / 3.0;
    x += -100.0 + 2.0 * longitude + 3.0 * latitude + 0.2 * latitude * latitude + 0.1 * longlat + 0.2 * abs_x;
    y += 300.0 + longitude + 2.0 * latitude + 0.1 * longitude * longitude + 0.1 * longlat + 0.1 * abs_x;

    *out_latitude = x;
    *out_longitude = y;
}

static void
_delta(double longitude, double latitude, double *out_longitude, double *out_latitude) {
    double d_lat, d_long;
    _transform(longitude - 105.0, latitude - 35.0, &d_long, &d_lat);

    double rad_latitude = latitude / 180.0 * PI;
    double magic = sin(rad_latitude);
    magic = 1 - OFFSET * magic * magic;
    double sqrt_magic = sqrt(magic);

    d_lat = (d_lat * 180.0) / ((AXIS * (1 - OFFSET)) / (magic * sqrt_magic) * PI);
    d_long = (d_long * 180.0) / (AXIS / sqrt_magic * cos(rad_latitude) * PI);

    *out_latitude = latitude + d_lat;
    *out_longitude = longitude + d_long;
}

int
nmea_in_china(double longitude, double latitude) {
    if (longitude < 72.004 || longitude > 135.05) {
        return 0;
    }
    if (latitude < 3.86 || latitude > 53.55) {
        return 0;
    }
    return 1;
}

void
nmea_bd09_gcj02(double longitude, double latitude, double *g_longitude, double *g_latitude) {
    double x = longitude - 0.0065;
    double y = latitude - 0.006;

    double z = sqrt(x * x + y * y) - 0.00002 * sin(y * X_PI);
    double theta = atan2(y, x) - 0.000003 * cos(x * X_PI);

    *g_longitude = z * cos(theta);
    *g_latitude = z * sin(theta);
}

void
nmea_gcj02_bd09(double longitude, double latitude, double *b_longitude, double *b_latitude) {
    double z = sqrt(longitude * longitude + latitude * latitude) + 0.00002 * sin(latitude * X_PI);
    double theta = atan2(latitude, longitude) + 0.000003 * cos(longitude * X_PI);

    *b_longitude = z * cos(theta) + 0.0065;
    *b_latitude = z * sin(theta) + 0.006;
}

void
nmea_wgs84_gcj02(double longitude, double latitude, double *g_longitude, double *g_latitude) {
    if (!nmea_in_china(longitude, latitude)) {
        *g_longitude = longitude;
        *g_latitude = latitude;
        return;
    }
    _delta(longitude, latitude, g_longitude, g_latitude);
}

void
nmea_gcj02_wgs84(double longitude, double latitude, double *w_longitude, double *w_latitude) {
    double m_longitude, m_latitude;

    if (!nmea_in_china(longitude, latitude)) {
        *w_longitude = longitude;
        *w_latitude = latitude;
        return;
    }

    _delta(longitude, latitude, &m_longitude, &m_latitude);

    *w_longitude = longitude * 2 - m_longitude;
    *w_latitude = latitude * 2 - m_latitude;
}

void
nmea_bd09_wgs84(double longitude, double latitude, double *w_longitude, double *w_latitude) {
    double g_longitude, g_latitude;
    nmea_bd09_gcj02(longitude, latitude, &g_longitude, &g_latitude);
    nmea_gcj02_wgs84(g_longitude, g_latitude, w_longitude, w_latitude);
}

void
nmea_wgs84_bd09(double longitude, double latitude, double *b_longitude, double *b_latitude) {
    double g_longitude, g_latitude;
    nmea_wgs84_gcj02(longitude, latitude, &g_longitude, &g_latitude);
    nmea_gcj02_bd09(g_longitude, g_latitude, b_longitude, b_latitude);
}

#endif /* NMEA_0183_IMPLEMENTATION */
