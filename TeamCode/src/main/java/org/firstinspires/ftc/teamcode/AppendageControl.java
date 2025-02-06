package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class AppendageControl {

    private static double DUNK_AUTO_RETRACT_WHEN_SCORE_THRESHOLD = TerabytesIntoTheDeep.TILT_DUNK_RANGE / 3;
    private static double DUNK_AUTO_RETRACT_DELAY = 250;
    private static double DISTANCE_SIGNAL_INCREMENT_AMOUNT = 0.085;
    private static int ARM_SETTLED_TICK_THRESHOLD = 32;
    private static int EXTENDER_SETTLED_TICK_THRESHOLD = 24;
    private static int UNTUCK_END_EFFECTOR_TIMEOUT_MILLIS = 3000;

    // NEW: For vision-based wrist control debounce.
    private boolean waitingForWristSettle = false;
    private ElapsedTime wristSettleTimer = new ElapsedTime();
    private static final double WRIST_SETTLE_TIME_MS = 300; // milliseconds delay until we accept new vision input
    private static final double WRIST_SERVO_TOLERANCE = 0.02; // not used if no feedback is available

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
    private ElapsedTime untuckedTimer;

    public AppendageControl(AppendageControlState initialState, boolean isAuton) {
        currentState = initialState;
        this.isAuton = isAuton;
    }

    public AppendageControlTarget evaluate(int armLTicks, int armRTicks, int extenderTicks) {
        currentArmLTicks = armLTicks;
        currentArmRTicks = armRTicks;
        currentExtenderTicks = extenderTicks;

        if (justDunkedTimer != null && justDunkedTimer.milliseconds() > DUNK_AUTO_RETRACT_DELAY) {
            if (isBasketScoring()) {
                setControlState(AppendageControlState.DEFENSIVE);
            }
            justDunkedTimer = null;
        }

        // NEW: Check if the wrist servo has had enough time to settle.
        if (waitingForWristSettle && wristSettleTimer.milliseconds() > WRIST_SETTLE_TIME_MS) {
            waitingForWristSettle = false;
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
            case COLLECT_SAFE:
                evaluateCollectSafe();
                break;
            case LOW_BASKET:
                evaluateLowBasket();
                break;
            case HIGH_BASKET:
                evaluateHighBasket();
                break;
            case COLLECT_CLIP:
                evaluateCollectClip();
                break;
            case SCORE_CLIP:
                evaluateScoreClip();
                break;
            case HANG:
                evaluateHang();
            default:
                throw new IllegalArgumentException("Unexpected state: " + currentState);
        }

        return target;
    }

    public void updateVisionWristAdjustment(Double wristHeadingErrorDegrees) {
        double wristOffsetTicks = TerabytesIntoTheDeep.WRIST_ORIGIN - target.wristTarget;
        double currentWristHeadingOffset = wristOffsetTicks * TerabytesIntoTheDeep.WRIST_DEGREES_TOTAL_RANGE;
        if (currentState == AppendageControlState.COLLECTING && !waitingForWristSettle) {
            double wristHeadingDegreesToSet =
                    Math.max(TerabytesIntoTheDeep.WRIST_DEGREES_HEADING_MIN,
                            Math.min(TerabytesIntoTheDeep.WRIST_DEGREES_HEADING_MAX,
                                    currentWristHeadingOffset + wristHeadingErrorDegrees));

            if (Math.abs(wristHeadingDegreesToSet) > TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE) {
                wristHeadingDegreesToSet = (180 - wristHeadingDegreesToSet) % 180;
            }

            wristSignal = wristHeadingDegreesToSet / TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE;
            waitingForWristSettle = true;
            wristSettleTimer.reset();
        }
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
        if (isBasketScoring() && dunkSignal > DUNK_AUTO_RETRACT_WHEN_SCORE_THRESHOLD) {
            justDunkedTimer = new ElapsedTime();
        }
    }

    public void setWristSignal(double signal) {
        wristSignal = signal;
    }

    public void setDistanceSignal(double signal) {
        collectDistanceSignal = signal;
    }

    public void setHeightSignal(double signal) {
        collectHeightSignal = signal;
    }

    public void setDunkSignal(double signal) {
        dunkSignal = signal;
    }

    public void applyTiltLevel(boolean level) {
        levelTilt = level;
    }

    public void setControlState(AppendageControlState newState) {
        if (this.currentState == AppendageControlState.TUCKED && newState != AppendageControlState.TUCKED) {
            untuckedTimer = new ElapsedTime();
        }
        previousState = currentState;
        currentState = newState;
    }

    public void resetCollectParametersToDefault() {
        collectDistanceSignal = 0;
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
        if (untuckedTimer != null && untuckedTimer.milliseconds() < UNTUCK_END_EFFECTOR_TIMEOUT_MILLIS) {
            target.wristTarget = TerabytesIntoTheDeep.WRIST_TUCKED;
            target.tiltTarget = TerabytesIntoTheDeep.TILT_TUCKED;
            target.pincerTarget = TerabytesIntoTheDeep.PINCER_OPEN;
            return;
        }

        double armDegreesFromHorizontal = currentArmDegreesAboveHorizontal();
        double tiltLevel = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 - armDegreesFromHorizontal));
        double tiltClip = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 + 35 - armDegreesFromHorizontal));
        double tiltUp = tiltLevel + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * 90);
        double tiltDown = TerabytesIntoTheDeep.TILT_ORIGIN - (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * armDegreesFromHorizontal);
        boolean isClipCollect = currentState == AppendageControlState.COLLECT_CLIP;
        boolean isClipScore = currentState == AppendageControlState.SCORE_CLIP;
        boolean isCollecting = currentState == AppendageControlState.COLLECTING;
        double tiltActualSetpoint = isCollecting ? tiltDown : (isClipCollect || isClipScore) ? tiltClip : tiltUp;
        if (isBasketScoring() || isClipCollect) {
            tiltActualSetpoint = tiltActualSetpoint - (dunkSignal * TerabytesIntoTheDeep.TILT_DUNK_RANGE);
        } else if (isClipScore) {
            tiltActualSetpoint = tiltActualSetpoint - (dunkSignal * 0.5 * TerabytesIntoTheDeep.TILT_DUNK_RANGE);
        }
        tiltActualSetpoint = Math.max(0, Math.min(1, tiltActualSetpoint));
        double wristActualSetpoint = Math.max(-1, Math.min(1, wristSignal));
        target.wristTarget = isCollecting ? TerabytesIntoTheDeep.WRIST_ORIGIN + (TerabytesIntoTheDeep.WRIST_RANGE * wristActualSetpoint) : TerabytesIntoTheDeep.WRIST_ORIGIN;
        target.tiltTarget = levelTilt ? tiltLevel : tiltActualSetpoint;
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

    private void evaluateCollectSafe() {
        levelTilt = true;
        collectDistanceSignal = 0;
        collectHeightSignal = 1;
        evaluateCollecting();
    }

    private void evaluateLowBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES / 2);
        evaluateEndEffector();
    }

    private void evaluateHighBasket() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_BASKET_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES);
        evaluateEndEffector();
    }

    private void evaluateCollectClip() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_COLLECT_CLIP_ANGLE, 0);
        evaluateEndEffector();
    }
    private void evaluateScoreClip() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_SCORE_CLIP_ANGLE, 0);
        evaluateEndEffector();
    }
    private void evaluateHang(){
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.EXTENDER_HANG, TerabytesIntoTheDeep.ARM_HANG_ANGLE);
        evaluateEndEffector();
    }

    public double currentArmDegreesAboveHorizontal() {
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;
        double armDegreesFromZero = averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE;
        return armDegreesFromZero - TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
    }

    public boolean isBasketScoring() {
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
