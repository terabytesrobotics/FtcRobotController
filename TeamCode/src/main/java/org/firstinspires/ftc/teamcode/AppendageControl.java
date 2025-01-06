package org.firstinspires.ftc.teamcode;

class AppendageControl {

    private static int ARM_SETTLED_TICK_THRESHOLD = 32;
    private static int EXTENDER_SETTLED_TICK_THRESHOLD = 24;

    private int currentArmLTicks;
    private int currentArmRTicks;
    private int currentExtenderTicks;

    public AppendageControlState currentState;
    public final AppendageControlTarget target = new AppendageControlTarget(0, 0, TerabytesIntoTheDeep.TILT_ORIGIN, TerabytesIntoTheDeep.WRIST_ORIGIN, TerabytesIntoTheDeep.PINCER_CENTER);

    private double collectHeightSignal = 0.5d;
    private double collectDistanceSignal = 0d;
    private double dunkSignal = 0;
    private boolean openPincer = false;

    public AppendageControl(AppendageControlState initialState) {
        currentState = initialState;
    }

    public AppendageControlTarget evaluate(int armLTicks, int armRTicks, int extenderTicks) {
        currentArmLTicks = armLTicks;
        currentArmRTicks = armRTicks;
        currentExtenderTicks = extenderTicks;

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

    public void setPincerOpen(boolean open) {
        openPincer = open;
    }

    public void togglePincer() {
        openPincer = !openPincer;
    }

    public void setDunkSignal(double signal) {
        dunkSignal = signal;
    }

    public void setControlState(AppendageControlState state) {
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

    private void evaluateEndEffector() {
        double armDegreesFromHorizontal = currentArmDegreesAboveHorizontal();
        //double tiltLevel = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 - armDegreesFromHorizontal));
        double tiltUp = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (180 - armDegreesFromHorizontal));
        double tiltDown = TerabytesIntoTheDeep.TILT_ORIGIN - (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * armDegreesFromHorizontal);
        boolean isCollecting = currentState == AppendageControlState.COLLECTING;
        double tiltActualSetpoint = isCollecting ? tiltDown : tiltUp;
        if (isScoring()) {
            tiltActualSetpoint = tiltActualSetpoint + (dunkSignal * TerabytesIntoTheDeep.TILT_DUNK_RANGE);
        }
        tiltActualSetpoint = Math.max(0, Math.min(1, tiltActualSetpoint));
        target.wristTarget = TerabytesIntoTheDeep.WRIST_ORIGIN;
        target.tiltTarget = tiltActualSetpoint;
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

    public double currentEndEffectorOffsetFromRobotCenter() {
        if (currentState != AppendageControlState.COLLECTING) return 0;
        double extensionInches = currentExtenderTicks / TerabytesIntoTheDeep.EXTENDER_TICKS_PER_INCH;
        double totalExtension = TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES + extensionInches;
        double horizontalOffset = totalExtension * Math.cos(Math.toRadians(currentArmDegreesAboveHorizontal()));
        return horizontalOffset - TerabytesIntoTheDeep.ARM_AXLE_OFFSET_FROM_ROBOT_CENTER_INCHES;
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
