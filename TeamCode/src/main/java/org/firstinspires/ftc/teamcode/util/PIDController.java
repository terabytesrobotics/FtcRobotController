package org.firstinspires.ftc.teamcode.util;

// https://en.wikipedia.org/wiki/PID_controller
// https://github.com/CGrassin/SimplyPID/
public class PIDController {
    private static final double MIN_DT_SECONDS = 1e-4;

    public double kP, kI, kD;
    private double integral;
    private double lastError;
    private boolean hasLastError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current, double dtSeconds) {
        double error = target - current;
        double safeDtSeconds = Math.max(MIN_DT_SECONDS, dtSeconds);
        integral += error * safeDtSeconds;

        // fix integral windup
        double integralLimit = Double.POSITIVE_INFINITY;
        integral = Math.max(-integralLimit,
                Math.min(integral, integralLimit));

        double der = hasLastError ? (error - lastError) / safeDtSeconds : 0.0;
        lastError = error;
        hasLastError = true;

        return (kP * error) + (kI * integral) + (kD * der);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLastError = false;
    }
}
