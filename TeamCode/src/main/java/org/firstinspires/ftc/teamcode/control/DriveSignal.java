package org.firstinspires.ftc.teamcode.control;

/** Robot-relative drive request: +forward, +left, and +counterclockwise. */
public final class DriveSignal {
    public static final DriveSignal ZERO = new DriveSignal(0.0, 0.0, 0.0);

    private final double forward;
    private final double left;
    private final double counterclockwise;

    public DriveSignal(double forward, double left, double counterclockwise) {
        this.forward = forward;
        this.left = left;
        this.counterclockwise = counterclockwise;
    }

    public double getForward() {
        return forward;
    }

    public double getLeft() {
        return left;
    }

    public double getCounterclockwise() {
        return counterclockwise;
    }
}
