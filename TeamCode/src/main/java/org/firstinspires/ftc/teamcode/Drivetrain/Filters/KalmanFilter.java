package org.firstinspires.ftc.teamcode.Drivetrain.Filters;

import com.acmerobotics.dashboard.config.Config;

@Config
public class KalmanFilter {
    // Covariances. Note the the "model" covariance
    // could also be a covariance associated with a
    // sensor
    double Q; // model covariance
    double R; // sensor covariance

    double x; // state

    public static double p = 1;
    public static double K = 0.5; // Kalman gain

    double p_prev = p;
    double x_prev = x;

    /**
     * Kalman filter class
     * @param x_0
     * @param Q
     * @param R
     */
    public KalmanFilter(double x_0, double Q, double R) {
        this.x = x_0;
        this.Q = Q;
        this.R = R;
    }

    public double predictAndUpdate(double u, double z) {
        // Predict
        x = x_prev + u;
        p = p_prev + Q;
        K = p / (p + R);


        // Update
        x = x + K * (z - x);
        p = (1 - K) * p;

        // Set current values to new previous values
        x_prev = x;
        p_prev = p;

        return x;
    }
}
