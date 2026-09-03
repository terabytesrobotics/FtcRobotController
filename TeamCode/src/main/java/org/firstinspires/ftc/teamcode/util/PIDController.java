package org.firstinspires.ftc.teamcode.util;

// https://en.wikipedia.org/wiki/PID_controller
// https://github.com/CGrassin/SimplyPID/
public class PIDController {
    public double kP, kI, kD;
    private double integral;
    private double lastError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        integral += error;

        // fix integral windup
        double integralLimit = Double.POSITIVE_INFINITY;
        integral = Math.max(-integralLimit,
                Math.min(integral, integralLimit));

        double der = error - lastError;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * der);
    }
}
