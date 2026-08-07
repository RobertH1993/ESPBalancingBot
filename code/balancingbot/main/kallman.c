#include "kallman.h"

void kallman_init(kallman_filter_t *kf, float initial_angle){
    // Reasonable defaults for a MEMS accel/gyro combo, tune with the setters
    // below if the estimate is too noisy (increase R_measure) or too slow to
    // react (decrease R_measure / increase Q_angle).
    kf->Q_angle = 0.00001f;
    kf->Q_bias = 0.001f;
    kf->R_measure = 0.5f;

    kf->angle = initial_angle;
    kf->bias = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float kallman_update(kallman_filter_t *kf, float new_angle, float new_rate, float dt){
    // Predict: integrate the bias-corrected gyro rate to move the angle estimate forward
    float rate = new_rate - kf->bias;
    kf->angle += dt * rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Update: correct the prediction using the accelerometer-derived angle
    float S = kf->P[0][0] + kf->R_measure; // Estimate error
    float K0 = kf->P[0][0] / S; // Kalman gain for the angle state
    float K1 = kf->P[1][0] / S; // Kalman gain for the bias state

    float y = new_angle - kf->angle; // Innovation (measurement residual)

    kf->angle += K0 * y;
    kf->bias += K1 * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
}

void kallman_set_angle_variance(kallman_filter_t *kf, float q_angle){
    kf->Q_angle = q_angle;
}

void kallman_set_bias_variance(kallman_filter_t *kf, float q_bias){
    kf->Q_bias = q_bias;
}

void kallman_set_measurement_variance(kallman_filter_t *kf, float r_measure){
    kf->R_measure = r_measure;
}
