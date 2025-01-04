package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class AppendageControl {

    private int currentArmLTicks;
    private int currentArmRTicks;
    private int currentExtenderTicks;
    public AppendageControlState currentState;
    public final AppendageControlTarget target = new AppendageControlTarget(0, 0, TerabytesIntoTheDeep.TILT_ORIGIN, TerabytesIntoTheDeep.WRIST_ORIGIN, TerabytesIntoTheDeep.PINCER_CENTER);

    private double collectDistance = 0d;
    private boolean collect = false;
    private boolean openPincer = false;
    private boolean tiltTuck = false;
    private ElapsedTime untuckTimer = null;

    public AppendageControl() {
        currentState = AppendageControlState.TUCKED;
    }

    public AppendageControlTarget evaluate(int armLTicks, int armRTicks, int extenderTicks) {
        currentArmLTicks = armLTicks;
        currentArmRTicks = armRTicks;
        currentExtenderTicks = extenderTicks;

        if (untuckTimer != null) return evaluateUntucking();

        switch (currentState) {
            case TUCKED:
                return evaluateTucked();
            case DEFENSIVE:
                return evaluateDefensive();
            case COLLECTING:
                return evaluateCollecting();
            case LOW_BASKET:
                return evaluateLowBasket();
            case HIGH_BASKET:
                return evaluateHighBasket();
            default:
                throw new IllegalArgumentException("Unexpected state: " + currentState);
        }
    }

    public void applyTuck(boolean shouldTiltTuck) {
        tiltTuck = shouldTiltTuck;
    }

    public void applyOpenPincer(boolean shouldOpenPincer) {
        openPincer = shouldOpenPincer;
    }

    public void applyCollect(boolean shouldCollect) {
        collect = shouldCollect;
    }

    public void setControlState(AppendageControlState state) {
        if (currentState == AppendageControlState.TUCKED) {
            untuckTimer = new ElapsedTime();
        }
        currentState = state;
    }

    public void accumulateCollectDistance(double collectDistance) {
        this.collectDistance += collectDistance;
        this.collectDistance = Math.max(0, Math.min(1, this.collectDistance));
    }

    private AppendageControlTarget evaluateUntucking() {
        double t = untuckTimer.milliseconds();
        if (t < 2000) {
            target.armTickTarget = TerabytesIntoTheDeep.ARM_LEVEL_TICKS;
            target.tiltTarget = TerabytesIntoTheDeep.TILT_TUCKED;
            target.wristTarget = TerabytesIntoTheDeep.WRIST_TUCKED;
            target.pincerTarget = TerabytesIntoTheDeep.PINCER_CLOSED;
        } else {
            untuckTimer = null;
        }
        return target;
    }

    private void evaluateEndEffector() {
        double armDegreesFromHorizontal = currentArmDegreesAboveHorizontal();
        double tiltLevel = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 - armDegreesFromHorizontal));
        double tiltUp = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (180 - armDegreesFromHorizontal));
        double tiltDown = TerabytesIntoTheDeep.TILT_ORIGIN - (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * armDegreesFromHorizontal);
        boolean isCollecting = currentState == AppendageControlState.COLLECTING;
        double tiltDefaultSetpoint = isCollecting ? tiltLevel : tiltUp;
        double tiltSetpoint = isCollecting && collect && armDegreesFromHorizontal < 0 ? tiltDown : tiltDefaultSetpoint;
        target.wristTarget = TerabytesIntoTheDeep.WRIST_ORIGIN;
        target.tiltTarget = tiltTuck ? TerabytesIntoTheDeep.TILT_TUCKED : tiltSetpoint;
        target.pincerTarget = openPincer ? TerabytesIntoTheDeep.PINCER_OPEN : TerabytesIntoTheDeep.PINCER_CLOSED;
    }

    private void setArmAndExtenderSetpoints(double angleAboveHorizontal, double inchesExtension) {
        target.armTickTarget = TerabytesIntoTheDeep.ARM_LEVEL_TICKS + (angleAboveHorizontal * TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE);
        target.extenderTickTarget = inchesExtension * TerabytesIntoTheDeep.EXTENDER_TICS_PER_INCH;
    }

    private AppendageControlTarget evaluateTucked() {
        target.armTickTarget = 0;
        target.extenderTickTarget = 0;
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateDefensive() {
        setArmAndExtenderSetpoints(90, 0);
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateCollecting() {
        double clampedCollectDistance = Math.max(0, Math.min(1, collectDistance));
        double minimumAchievableDistance = Math.sqrt((TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES * TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES) - (TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES * TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES));
        double maximumAchievableDistance = Math.sqrt((TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH * TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH) - (TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES * TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES));
        double desiredDistance = minimumAchievableDistance + (clampedCollectDistance * (maximumAchievableDistance - minimumAchievableDistance));
        double desiredArmAngle = -Math.toDegrees(Math.atan2(TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES, desiredDistance));
        double extensionLengthToApply = 0;
        double currentArmAngle = currentArmDegreesAboveHorizontal();
        if (currentArmAngle < 0) {
            double currentImpliedDistance = TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES / Math.sin(Math.toRadians(-currentArmAngle));
            double currentImpliedTotalLength = Math.sqrt((TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES * TerabytesIntoTheDeep.ARM_COLLECT_DEPTH_INCHES) + (currentImpliedDistance * currentImpliedDistance));
            extensionLengthToApply = Math.max(0, TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH - currentImpliedTotalLength);
        }

        setArmAndExtenderSetpoints(desiredArmAngle, extensionLengthToApply);
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateLowBasket() {
        setArmAndExtenderSetpoints(105, 6);
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateHighBasket() {
        setArmAndExtenderSetpoints(90, 14);
        evaluateEndEffector();
        return target;
    }

    // Negative when collecting
    public double currentArmDegreesAboveHorizontal() {
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;
        double armDegreesFromZero = averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE;
        return armDegreesFromZero + TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
    }
}
