package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

class AppendageControl {

    private int currentArmLTicks;
    private int currentArmRTicks;
    private int currentExtenderTicks;

    public AppendageControlState currentState;
    public final AppendageControlTarget target = new AppendageControlTarget(0, 0, TerabytesIntoTheDeep.TILT_ORIGIN, TerabytesIntoTheDeep.WRIST_ORIGIN, TerabytesIntoTheDeep.PINCER_CENTER);

    private double collectHeightSignal = 0.5d;
    private double collectDistanceSignal = 0d;
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

    public void setPincerOpen(boolean open) {
        openPincer = open;
    }

    public void togglePincer() {
        openPincer = !openPincer;
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

    public void resetCollectDistance() {
        collectDistanceSignal = 0;
    }

    public void resetCollectHeight() {
        collectHeightSignal = 0.5;
    }

    public void accumulateCollectDistanceSignal(double collectDistance) {
        this.collectDistanceSignal += collectDistance;
        this.collectDistanceSignal = Math.max(0, Math.min(1, this.collectDistanceSignal));
    }

    public void accumulateCollectHeightSignal(double collectHeight) {
        this.collectHeightSignal += collectHeight;
        this.collectHeightSignal = Math.max(0, Math.min(1, this.collectHeightSignal));
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
        target.extenderTickTarget = inchesExtension * TerabytesIntoTheDeep.EXTENDER_TICKS_PER_INCH;
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
        double clampedCollectHeightSignal = Math.max(0, Math.min(1, collectHeightSignal));
        double desiredCollectHeight =
                TerabytesIntoTheDeep.ARM_MIN_COLLECT_HEIGHT_INCHES +
                        (clampedCollectHeightSignal * (TerabytesIntoTheDeep.ARM_MAX_COLLECT_HEIGHT_INCHES - TerabytesIntoTheDeep.ARM_MIN_COLLECT_HEIGHT_INCHES));
        double desiredCollectDepth = TerabytesIntoTheDeep.ARM_AXLE_HEIGHT_INCHES - desiredCollectHeight;
        double clampedCollectDistanceSignal = Math.max(0, Math.min(1, collectDistanceSignal));
        double minimumAchievableDistanceInches = Math.sqrt((TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES * TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES) - (desiredCollectDepth * desiredCollectDepth));
        double maximumAchievableDistanceInches = Math.sqrt((TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH * TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH) - (desiredCollectDepth * desiredCollectDepth));
        double desiredDistance = minimumAchievableDistanceInches + (clampedCollectDistanceSignal * (maximumAchievableDistanceInches - minimumAchievableDistanceInches));
        double desiredArmAngle = -Math.toDegrees(Math.atan2(desiredCollectDepth, desiredDistance));
        double extensionLengthToApply = 0;
        double currentArmAngle = currentArmDegreesAboveHorizontal();
        if (currentArmAngle < 0) {
            double currentImpliedDistance = desiredCollectDepth / Math.tan(Math.toRadians(-currentArmAngle));
            double currentImpliedTotalLength = Math.sqrt((desiredCollectDepth * desiredCollectDepth) + (currentImpliedDistance * currentImpliedDistance));
            double limitedCurrentImpliedTotalLength = Math.min(TerabytesIntoTheDeep.EXTENDER_MAX_TOTAL_LENGTH, currentImpliedTotalLength);
            extensionLengthToApply = Math.max(0, limitedCurrentImpliedTotalLength - TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES);
        }

        setArmAndExtenderSetpoints(desiredArmAngle, extensionLengthToApply);
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateLowBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES / 2);
        evaluateEndEffector();
        return target;
    }

    private AppendageControlTarget evaluateHighBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES);
        evaluateEndEffector();
        return target;
    }

    // Negative when collecting
    public double currentArmDegreesAboveHorizontal() {
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;
        double armDegreesFromZero = averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE;
        return armDegreesFromZero - TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
    }

    public double currentEndEffectorOffsetFromRobotCenter() {
        if (currentState != AppendageControlState.COLLECTING) return 0;
        double extensionInches = currentExtenderTicks / TerabytesIntoTheDeep.EXTENDER_TICKS_PER_INCH;
        double totalExtension = TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES + extensionInches;
        double horizontalOffset = totalExtension * Math.cos(Math.toRadians(currentArmDegreesAboveHorizontal()));
        return horizontalOffset - TerabytesIntoTheDeep.ARM_AXLE_OFFSET_FROM_ROBOT_CENTER_INCHES;
    }
}
