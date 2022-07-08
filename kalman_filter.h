/**
 * @file kalman_filter.h
 * @author zhoukk (izhoukk@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 * http://en.volupedia.org/wiki/Kalman_filter
 */

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "matrix.h"

/* 1 Dimension */
typedef struct {
    double x; /* state */
    double A; /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    double H; /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    double q; /* process(predict) noise convariance */
    double r; /* measure noise convariance */
    double p; /* estimated error convariance */
    double gain;
} kalman_filter_1d_t;

/* 2 Dimension */
typedef struct {
    double x[2];    /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    double A[2][2]; /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    double H[2];    /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    double q[2];    /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    double r;       /* measure noise convariance */
    double p[2][2]; /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    double gain[2]; /* 2x1 */
} kalman_filter_2d_t;

void kalman_filter_1d_init(kalman_filter_1d_t *state, double init_x, double init_p);
double kalman_filter_1d(kalman_filter_1d_t *state, double z_measure);

void kalman_filter_2d_init(kalman_filter_2d_t *state, double *init_x, double (*init_p)[2]);
double kalman_filter_2d(kalman_filter_2d_t *state, double z_measure);

typedef struct {

    int state_dimension;
    int observation_dimension;

    /* k */
    int timestep;

    /* F_k */
    matrix_t state_transition;
    /* H_k */
    matrix_t observation_model;
    /* Q_k */
    matrix_t process_noise_covariance;
    /* R_k */
    matrix_t observation_noise_covariance;

    /* z_k */
    matrix_t observation;

    /* x-hat_k|k-1 */
    matrix_t predicted_state;
    /* P_k|k-1 */
    matrix_t predicted_estimate_covariance;
    /* y-tilde_k */
    matrix_t innovation;
    /* S_k */
    matrix_t innovation_covariance;
    /* S_k^-1 */
    matrix_t inverse_innovation_covariance;
    /* K_k */
    matrix_t optimal_gain;
    /* x-hat_k|k */
    matrix_t state_estimate;
    /* P_k|k */
    matrix_t estimate_covariance;

    matrix_t vertical_scratch;
    matrix_t small_square_scratch;
    matrix_t big_square_scratch;

} kalman_filter_t;

kalman_filter_t *kalman_filter_new(int state_dimension, int observation_dimension);

void kalman_filter_free(kalman_filter_t *kf);

void kalman_filter_update(kalman_filter_t *kf);

#endif // _KALMAN_FILTER_H_

#ifdef KALMAN_FILTER_IMPLEMENTATION

void
kalman_filter_1d_init(kalman_filter_1d_t *state, double init_x, double init_p) {
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2; // 10e-6;  /* predict noise convariance */
    state->r = 5e2; // 10e-5;  /* measure error convariance */
}

double
kalman_filter_1d(kalman_filter_1d_t *state, double z_measure) {
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q; /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

void
kalman_filter_2d_init(kalman_filter_2d_t *state, double *init_x, double (*init_p)[2]) {
    state->x[0] = init_x[0];
    state->x[1] = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    // state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    // state->H       = {1,0};
    state->H[0] = 1;
    state->H[1] = 0;
    // state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0] = 10e-7;
    state->q[1] = 10e-7;
    state->r = 10e-7; /* estimated error convariance */
}

double
kalman_filter_2d(kalman_filter_2d_t *state, double z_measure) {
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp);
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}

kalman_filter_t *
kalman_filter_new(int state_dimension, int observation_dimension) {
    kalman_filter_t *kf = (kalman_filter_t *)malloc(sizeof *kf);

    kf->timestep = 0;
    kf->state_dimension = state_dimension;
    kf->observation_dimension = observation_dimension;

    kf->state_transition = matrix_new(state_dimension, state_dimension);
    kf->observation_model = matrix_new(observation_dimension, state_dimension);
    kf->process_noise_covariance = matrix_new(state_dimension, state_dimension);
    kf->observation_noise_covariance = matrix_new(observation_dimension, observation_dimension);

    kf->observation = matrix_new(observation_dimension, 1);

    kf->predicted_state = matrix_new(state_dimension, 1);
    kf->predicted_estimate_covariance = matrix_new(state_dimension, state_dimension);
    kf->innovation = matrix_new(observation_dimension, 1);
    kf->innovation_covariance = matrix_new(observation_dimension, observation_dimension);
    kf->inverse_innovation_covariance = matrix_new(observation_dimension, observation_dimension);
    kf->optimal_gain = matrix_new(state_dimension, observation_dimension);
    kf->state_estimate = matrix_new(state_dimension, 1);
    kf->estimate_covariance = matrix_new(state_dimension, state_dimension);

    kf->vertical_scratch = matrix_new(state_dimension, observation_dimension);
    kf->small_square_scratch = matrix_new(observation_dimension, observation_dimension);
    kf->big_square_scratch = matrix_new(state_dimension, state_dimension);
}

void
kalman_filter_free(kalman_filter_t *kf) {
    matrix_free(kf->state_transition);
    matrix_free(kf->observation_model);
    matrix_free(kf->process_noise_covariance);
    matrix_free(kf->observation_noise_covariance);

    matrix_free(kf->observation);

    matrix_free(kf->predicted_state);
    matrix_free(kf->predicted_estimate_covariance);
    matrix_free(kf->innovation);
    matrix_free(kf->innovation_covariance);
    matrix_free(kf->inverse_innovation_covariance);
    matrix_free(kf->optimal_gain);
    matrix_free(kf->state_estimate);
    matrix_free(kf->estimate_covariance);

    matrix_free(kf->vertical_scratch);
    matrix_free(kf->small_square_scratch);
    matrix_free(kf->big_square_scratch);

    free(kf);
}

static void
kalman_filter_predict(kalman_filter_t *kf) {
    kf->timestep++;

    matrix_mul(kf->state_transition, kf->state_estimate, kf->predicted_state);

    matrix_mul(kf->state_transition, kf->estimate_covariance, kf->big_square_scratch);

    matrix_mul_transpose(kf->big_square_scratch, kf->state_transition, kf->predicted_estimate_covariance);

    matrix_add(kf->predicted_estimate_covariance, kf->process_noise_covariance, kf->predicted_estimate_covariance);
}

static void
kalman_filter_estimate(kalman_filter_t *kf) {
    matrix_mul(kf->observation_model, kf->predicted_state, kf->innovation);
    matrix_sub(kf->observation, kf->innovation, kf->innovation);

    matrix_mul_transpose(kf->predicted_estimate_covariance, kf->observation_model, kf->vertical_scratch);
    matrix_mul(kf->observation_model, kf->vertical_scratch, kf->innovation_covariance);
    matrix_add(kf->innovation_covariance, kf->observation_noise_covariance, kf->innovation_covariance);

    matrix_destructive_invert(kf->innovation_covariance, kf->inverse_innovation_covariance);

    matrix_mul(kf->vertical_scratch, kf->inverse_innovation_covariance, kf->optimal_gain);

    matrix_mul(kf->optimal_gain, kf->innovation, kf->state_estimate);
    matrix_add(kf->state_estimate, kf->predicted_state, kf->state_estimate);

    /* Estimate the state covariance */
    matrix_mul(kf->optimal_gain, kf->observation_model, kf->big_square_scratch);
    matrix_sub_identity(kf->big_square_scratch);
    matrix_mul(kf->big_square_scratch, kf->predicted_estimate_covariance, kf->estimate_covariance);
}

void
kalman_filter_update(kalman_filter_t *kf) {
    kalman_filter_predict(kf);
    kalman_filter_estimate(kf);
}

#endif // KALMAN_FILTER_IMPLEMENTATION