package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class AppendageControl {

    private static double DUNK_AUTO_RETRACT_WHEN_SCORE_THRESHOLD = TerabytesIntoTheDeep.TILT_DUNK_RANGE / 3;
    private static double DUNK_AUTO_RETRACT_DELAY = 250;
    private static double DISTANCE_SIGNAL_INCREMENT_AMOUNT = 0.085;
    private static int ARM_SETTLED_TICK_THRESHOLD = 32;
    private static int EXTENDER_SETTLED_TICK_THRESHOLD = 24;

    private int currentArmLTicks;
    private int currentArmRTicks;
    private int currentExtenderTicks;

    public AppendageControlState previousState;
    public AppendageControlState currentState;
    public final AppendageControlTarget target = new AppendageControlTarget(0, 0, TerabytesIntoTheDeep.TILT_ORIGIN, TerabytesIntoTheDeep.WRIST_ORIGIN, TerabytesIntoTheDeep.PINCER_CENTER);

    private double collectHeightSignal = 0.5d;
    private double collectDistanceSignal = 0d;
    private double dunkSignal = 0;
    private double wristSignal = 0;
    private boolean openPincer = false;
    private boolean levelTilt = false;
    private boolean isAuton = false;
    private ElapsedTime justDunkedTimer;

    public AppendageControl(AppendageControlState initialState, boolean isAuton) {
        currentState = initialState;
        this.isAuton = isAuton;
    }

    public AppendageControlTarget evaluate(int armLTicks, int armRTicks, int extenderTicks) {
        currentArmLTicks = armLTicks;
        currentArmRTicks = armRTicks;
        currentExtenderTicks = extenderTicks;

        if (justDunkedTimer != null && justDunkedTimer.milliseconds() > DUNK_AUTO_RETRACT_DELAY) {
            if (isScoring()) {
                setControlState(AppendageControlState.DEFENSIVE);
            }
            justDunkedTimer = null;
        }

        switch (currentState) {
            case TUCKED:
                evaluateTucked();
                break;
            case DEFENSIVE:
                evaluateDefensive();
                break;
            case COLLECTING:
                evaluateCollecting();
                break;
            case LOW_BASKET:
                evaluateLowBasket();
                break;
            case HIGH_BASKET:
                evaluateHighBasket();
                break;
            default:
                throw new IllegalArgumentException("Unexpected state: " + currentState);
        }

        return target;
    }

    public void accumulateWristSignal(double signal) {
        wristSignal += signal;
        wristSignal = Math.max(-1, Math.min(1, this.wristSignal));
    }

    public void setPincerOpen(boolean open) {
        openPincer = open;
    }

    public void togglePincer() {
        openPincer = !openPincer;
        if (isScoring() && dunkSignal > DUNK_AUTO_RETRACT_WHEN_SCORE_THRESHOLD) {
            justDunkedTimer = new ElapsedTime();
        }
    }

    public void setDunkSignal(double signal) {
        dunkSignal = signal;
    }

    public void applyTiltLevel(boolean level) {
        levelTilt = level;
    }

    public void setControlState(AppendageControlState state) {
        previousState = currentState;
        currentState = state;
    }

    public void resetCollectDistance() {
        collectDistanceSignal = 0;
    }

    public void resetCollectHeight() {
        collectHeightSignal = 0.5;
    }

    public void incrementCollectDistance(int increments) {
        this.collectDistanceSignal += increments * DISTANCE_SIGNAL_INCREMENT_AMOUNT;
        this.collectDistanceSignal = Math.max(0, Math.min(1, this.collectDistanceSignal));
    }

    public void accumulateCollectHeightSignal(double collectHeight) {
        this.collectHeightSignal += collectHeight;
        this.collectHeightSignal = Math.max(0, Math.min(1, this.collectHeightSignal));
    }

    private void evaluateEndEffector() {
        double armDegreesFromHorizontal = currentArmDegreesAboveHorizontal();
        double tiltLevel = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 - armDegreesFromHorizontal));
        double tiltUp = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (180 - armDegreesFromHorizontal));
        double tiltDown = TerabytesIntoTheDeep.TILT_ORIGIN - (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (armDegreesFromHorizontal - 15));
        boolean isCollecting = currentState == AppendageControlState.COLLECTING;
        double tiltActualSetpoint = isCollecting ? tiltDown : tiltUp;
        if (isScoring()) {
            tiltActualSetpoint = tiltActualSetpoint - (dunkSignal * TerabytesIntoTheDeep.TILT_DUNK_RANGE);
        }
        tiltActualSetpoint = Math.max(0, Math.min(1, tiltActualSetpoint));
        double wristActualSetpoint = Math.max(-1, Math.min(1, wristSignal));
        target.wristTarget = isCollecting ? TerabytesIntoTheDeep.WRIST_ORIGIN + (TerabytesIntoTheDeep.WRIST_RANGE * wristActualSetpoint) : TerabytesIntoTheDeep.WRIST_ORIGIN;
        target.tiltTarget = levelTilt ? tiltLevel :  tiltActualSetpoint;
        target.pincerTarget = openPincer ? TerabytesIntoTheDeep.PINCER_OPEN : TerabytesIntoTheDeep.PINCER_CLOSED;
    }

    private void setArmAndExtenderSetpoints(double angleAboveHorizontal, double inchesExtension) {
        target.armTickTarget = TerabytesIntoTheDeep.ARM_LEVEL_TICKS + (angleAboveHorizontal * TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE);
        target.extenderTickTarget = inchesExtension * TerabytesIntoTheDeep.EXTENDER_TICKS_PER_INCH;
    }

    private void evaluateTucked() {
        target.armTickTarget = 0;
        target.extenderTickTarget = 0;
        evaluateEndEffector();
    }

    private void evaluateDefensive() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_DEFENSIVE_ANGLE, 0);
        evaluateEndEffector();
    }

    private void evaluateCollecting() {
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
        double desiredTotalLength = Math.sqrt((desiredCollectDepth * desiredCollectDepth) + (desiredDistance * desiredDistance));
        double extensionLengthToApply = Math.max(0, desiredTotalLength - TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES);

        setArmAndExtenderSetpoints(desiredArmAngle, extensionLengthToApply);
        evaluateEndEffector();
    }

    private void evaluateLowBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES / 2);
        evaluateEndEffector();
    }

    private void evaluateHighBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES);
        evaluateEndEffector();
    }

    // Negative when collecting
    public double currentArmDegreesAboveHorizontal() {
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;
        double armDegreesFromZero = averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE;
        return armDegreesFromZero - TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
    }

    public boolean isScoring() {
        return currentState == AppendageControlState.HIGH_BASKET || currentState == AppendageControlState.LOW_BASKET;
    }

    public boolean isSettled() {
        boolean armLSettled = Math.abs(target.armTickTarget - currentArmLTicks) < ARM_SETTLED_TICK_THRESHOLD;
        boolean armRSettled = Math.abs(target.armTickTarget - currentArmRTicks) < ARM_SETTLED_TICK_THRESHOLD;
        boolean armSettled = armLSettled && armRSettled;
        boolean extenderSettled = Math.abs(target.extenderTickTarget - currentExtenderTicks) < EXTENDER_SETTLED_TICK_THRESHOLD;
        return armSettled && extenderSettled;
    }
}
