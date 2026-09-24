package org.firstinspires.ftc.teamcode.control;

/** Final normalized wheel powers produced from one robot-relative drive signal. */
public final class WheelPowers {
    public static final WheelPowers ZERO = new WheelPowers(0.0, 0.0, 0.0, 0.0);

    private final double frontLeft;
    private final double frontRight;
    private final double backLeft;
    private final double backRight;

    public WheelPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public double getFrontLeft() {
        return frontLeft;
    }

    public double getFrontRight() {
        return frontRight;
    }

    public double getBackLeft() {
        return backLeft;
    }

    public double getBackRight() {
        return backRight;
    }
}
