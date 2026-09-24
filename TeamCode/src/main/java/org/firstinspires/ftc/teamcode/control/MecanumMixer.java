package org.firstinspires.ftc.teamcode.control;

/** Pure mecanum kinematics shared by manual and closed-loop drive behaviors. */
public final class MecanumMixer {
    private MecanumMixer() {
    }

    public static WheelPowers mix(DriveSignal requestedSignal) {
        double forward = requestedSignal.getForward();
        double left = requestedSignal.getLeft();
        double counterclockwise = Math.max(
                -1.0, Math.min(1.0, requestedSignal.getCounterclockwise()));

        double translationMagnitude = Math.hypot(forward, left);
        if (translationMagnitude > 1.0) {
            forward /= translationMagnitude;
            left /= translationMagnitude;
        }

        double frontLeft = forward - left - counterclockwise;
        double frontRight = forward + left + counterclockwise;
        double backLeft = forward + left - counterclockwise;
        double backRight = forward - left + counterclockwise;

        double maximumRequestedPower = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight)));
        double scale = maximumRequestedPower > 1.0 ? 1.0 / maximumRequestedPower : 1.0;

        return new WheelPowers(
                frontLeft * scale,
                frontRight * scale,
                backLeft * scale,
                backRight * scale);
    }
}
