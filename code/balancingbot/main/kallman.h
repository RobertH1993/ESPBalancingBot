#ifndef _KALLMAN_H
#define _KALLMAN_H

// Simple 2-state Kalman filter (angle + gyro bias) used to fuse an
// accelerometer-derived angle with a gyroscope rate measurement into a
// single, low-noise angle estimate. This is the classic filter used in most
// self-balancing robot projects.
typedef struct{
    // Process noise variance: how much we trust the gyro-integrated angle
    // between measurements. Lower = smoother but slower to react.
    float Q_angle;
    // Process noise variance: how fast the estimated gyro bias is allowed to drift.
    float Q_bias;
    // Measurement noise variance: how much we trust the accelerometer angle.
    // Higher = smoother but slower to correct for gyro drift.
    float R_measure;

    // State: current angle estimate (deg) and estimated gyro bias (deg/s)
    float angle;
    float bias;

    // Error covariance matrix
    float P[2][2];
} kallman_filter_t;

// Initialize the filter. initial_angle should be a first accelerometer-derived
// angle measurement so the filter doesn't have to converge from zero.
void kallman_init(kallman_filter_t *kf, float initial_angle);

// Feed a new accelerometer angle (deg) and gyro rate (deg/s) sample into the
// filter and return the new filtered angle estimate (deg). Call this once
// per control loop iteration with the loop's dt (s).
float kallman_update(kallman_filter_t *kf, float new_angle, float new_rate, float dt);

// Tuning setters, only needed if the defaults set in kallman_init don't suit your sensor/setup
void kallman_set_angle_variance(kallman_filter_t *kf, float q_angle);
void kallman_set_bias_variance(kallman_filter_t *kf, float q_bias);
void kallman_set_measurement_variance(kallman_filter_t *kf, float r_measure);

#endif // _KALLMAN_H
