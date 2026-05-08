package org.firstinspires.ftc.teamcode.pedroPathing;

public class KalmanFilter {
    private double Q; // Process noise (Uncertainty added by purely relying on Odometry)
    private double R; // Measurement noise (Sensor noise from Limelight)
    private double p = 1.0; // Error covariance

    /**
     * Initializes the Kalman filter parameters.
     * @param Q Process noise (Drift associated with Pedro Pathing's continuous OTOS updates)
     * @param R Measurement noise (How noisy our Limelight measurements are)
     */
    public KalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
    }

    /**
     * Increases the uncertainty of our estimate proportioned to movement.
     * Call this continuously as the robot drives.
     * @param delta The distance traveled / change in value since last update.
     */
    public void predict(double delta) {
        p = p + (Q * Math.abs(delta));
    }

    /**
     * Fuses the current prediction (odometry pose) with the measurement (vision pose).
     * @param prediction The current smooth pose directly from Pedro Pathing
     * @param measurement The absolute pose from Limelight Megatag
     * @return The fused pose to firmly set back into Pedro Pathing
     */
    public double update(double prediction, double measurement) {
        // Calculate Kalman Gain
        double k = p / (p + R);
        
        // Fused State
        double x_new = prediction + k * (measurement - prediction);
        
        // Update Error Covariance
        p = (1 - k) * p;
        
        return x_new;
    }
}
