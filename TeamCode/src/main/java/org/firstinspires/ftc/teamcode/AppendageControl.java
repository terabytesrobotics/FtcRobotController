package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class AppendageControl {

    private static double DUNK_AUTO_RETRACT_WHEN_SCORE_THRESHOLD = TerabytesIntoTheDeep.TILT_DUNK_RANGE / 3;
    private static double DUNK_AUTO_RETRACT_DELAY = 250;
    private static double DISTANCE_SIGNAL_INCREMENT_AMOUNT = 0.085;
    private static int ARM_SETTLED_TICK_THRESHOLD = 32;
    private static int EXTENDER_SETTLED_TICK_THRESHOLD = 24;
    private static int UNTUCK_END_EFFECTOR_TIMEOUT_MILLIS = 750;

    private boolean waitingForWristSettle = false;
    private ElapsedTime wristSettleTimer = new ElapsedTime();
    private static final double WRIST_SETTLE_MIN_TIME = 100;
    private static final double WRIST_SETTLE_MS_PER_DEGREE = 10;
    private static final double WRIST_SETTLE_TIME_DEFAULT = 500;

    private int currentArmLTicks;
    private int currentArmRTicks;
    private int currentExtenderTicks;

    public AppendageControlState previousState;
    public AppendageControlState currentState;
    public volatile AppendageControlTarget target = new AppendageControlTarget(0, 0, TerabytesIntoTheDeep.TILT_ORIGIN, TerabytesIntoTheDeep.WRIST_ORIGIN, TerabytesIntoTheDeep.PINCER_CENTER);

    private double collectHeightSignal = 0.5d;
    private double collectDistanceSignal = 0d;
    private double dunkSignal = 0;
    private double wristSignal = 0;
    private boolean openPincer = false;
    private boolean levelTilt = false;
    private boolean isAuton = false;
    private ElapsedTime justDunkedTimer;
    private ElapsedTime untuckedTimer;

    private double wristDynamicTimeoutMs = WRIST_SETTLE_TIME_DEFAULT;

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

        if (waitingForWristSettle && wristSettleTimer.milliseconds() > wristDynamicTimeoutMs) {
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
            case CLIP_CLIP:
                evaluateClipClip();
                break;
            case PRE_HANG_1:
                evaluatePreHang1();
                break;
            case PRE_HANG_2:
                evaluatePreHang2();
                break;
            case HANG:
                evaluateHang();
                break;
            default:
                throw new IllegalArgumentException("Unexpected state: " + currentState);
        }

        return target;
    }

    private boolean shouldUpdateWristBasedOnVisionError() {
        boolean isCollectingCameraDown = currentState == AppendageControlState.COLLECTING;
        Double endEffectorHeight = getCurrentEndEffectorHeight();
        return isCollectingCameraDown &&
                endEffectorHeight != null &&
                (endEffectorHeight < TerabytesIntoTheDeep.ARM_MAX_HEIGHT_WRIST_DETECT_INCHES &&
                        endEffectorHeight > TerabytesIntoTheDeep.ARM_MIN_HEIGHT_WRIST_DETECT_INCHES);
    }

    private boolean shouldUpdateExtenderBasedOnVisionError() {
        boolean isCollectingCameraDown = currentState == AppendageControlState.COLLECTING;
        Double endEffectorHeight = getCurrentEndEffectorHeight();
        return isCollectingCameraDown &&
                endEffectorHeight != null &&
                (endEffectorHeight < TerabytesIntoTheDeep.ARM_MAX_HEIGHT_EXTENDER_DETECT_INCHES &&
                        endEffectorHeight > TerabytesIntoTheDeep.ARM_MIN_HEIGHT_EXTENDER_DETECT_INCHES);
    }

    public void updateVisionWristAdjustment(Double wristHeadingErrorDegrees) {
        if (wristHeadingErrorDegrees != null &&isWristMotionSettled() && shouldUpdateWristBasedOnVisionError()) {
            double wristOffsetTicks = target.wristTarget - TerabytesIntoTheDeep.WRIST_ORIGIN;
            double currentWristHeadingOffset = (wristOffsetTicks / TerabytesIntoTheDeep.WRIST_RANGE)
                    * TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE;


            // Compute the raw candidate angle by applying the vision error correction.
            double candidateAngle = currentWristHeadingOffset + wristHeadingErrorDegrees;

            // If candidateAngle is out of the allowed range, try the 180° equivalent.
            if (Math.abs(candidateAngle) > TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE) {
                double altCandidate = candidateAngle > 0 ? candidateAngle - 180 : candidateAngle + 180;
                // Use the alternative if it is closer to zero (i.e., within the allowed half range).
                if (Math.abs(altCandidate) < Math.abs(candidateAngle)) {
                    candidateAngle = altCandidate;
                }
            }

            // Clamp the candidate angle to be within the allowable half range.
            candidateAngle = Math.max(-TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE,
                    Math.min(TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE, candidateAngle));

            // Set the normalized wrist signal based on the candidate angle.
            wristSignal = candidateAngle / TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE;

            // Compute the actual adjustment required: the difference between the candidate angle and the current angle.
            double angleDifference = Math.abs(candidateAngle - currentWristHeadingOffset);

            // Compute a dynamic timeout proportional to the adjustment.
            // Ensure the timeout is at least WRIST_SETTLE_MIN_TIME.
            wristDynamicTimeoutMs = Math.max(WRIST_SETTLE_MIN_TIME, angleDifference * WRIST_SETTLE_MS_PER_DEGREE);

            waitingForWristSettle = true;
            wristSettleTimer.reset();
        }
    }

    public void updateVisionExtenderAdjustment(Double signal) {
        if (signal != null && isWristMotionSettled() && shouldUpdateExtenderBasedOnVisionError()) {
            accumulateCollectDistanceSignal(signal);
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
        collectHeightSignal = 1;
    }

    public void incrementCollectDistance(int increments) {
        this.collectDistanceSignal += increments * DISTANCE_SIGNAL_INCREMENT_AMOUNT;
        this.collectDistanceSignal = Math.max(0, Math.min(1, this.collectDistanceSignal));
    }

    public void accumulateCollectDistanceSignal(double collectDistance) {
        this.collectDistanceSignal += collectDistance;
        this.collectDistanceSignal = Math.max(0, Math.min(1, this.collectDistanceSignal));
    }

    public void accumulateCollectHeightSignal(double collectHeight) {
        this.collectHeightSignal += collectHeight;
        this.collectHeightSignal = Math.max(0, Math.min(1, this.collectHeightSignal));
    }

    private void holdSafe() {
        target.wristTarget = TerabytesIntoTheDeep.WRIST_TUCKED;
        target.tiltTarget = TerabytesIntoTheDeep.TILT_TUCKED;
        target.pincerTarget = TerabytesIntoTheDeep.PINCER_OPEN;
        target.armTickTarget = TerabytesIntoTheDeep.ARM_LEVEL_TICKS;
        target.extenderTickTarget = 0;
    }

    private void evaluateEndEffector() {
        double armDegreesFromHorizontal = currentArmDegreesAboveHorizontal();
        double tiltLevel = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 - armDegreesFromHorizontal));
        double tiltClip = TerabytesIntoTheDeep.TILT_ORIGIN + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * (90 + 35 - armDegreesFromHorizontal));
        double tiltUp = tiltLevel + (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * 90);
        double tiltDown = TerabytesIntoTheDeep.TILT_ORIGIN - (TerabytesIntoTheDeep.TILT_TICKS_PER_DEGREE * armDegreesFromHorizontal);
        boolean isClipCollect = currentState == AppendageControlState.COLLECT_CLIP;
        boolean isClipScore = currentState == AppendageControlState.SCORE_CLIP;
        boolean isClipClip = currentState == AppendageControlState.CLIP_CLIP;
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

        if (untuckedTimer != null && untuckedTimer.milliseconds() < UNTUCK_END_EFFECTOR_TIMEOUT_MILLIS) {
            holdSafe();
        } else {
            setArmAndExtenderSetpoints(desiredArmAngle, extensionLengthToApply);
            evaluateEndEffector();
        }
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
    private void evaluateClipClip() {
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_CLIP_CLIP_ANGLE, 0);
        evaluateEndEffector();
    }
    private void evaluatePreHang1(){
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_PRE_HANG_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES);
        evaluateEndEffector();
    }

    private void evaluatePreHang2(){
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_PRE_HANG_ANGLE, TerabytesIntoTheDeep.EXTENDER_MAX_EXTENSION_INCHES);
        evaluateEndEffector();
    }

    private void evaluateHang(){
        setArmAndExtenderSetpoints(TerabytesIntoTheDeep.ARM_HANG_ANGLE, TerabytesIntoTheDeep.EXTENDER_HANG);
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

    public double getCurrentWristHeadingDegrees() {
        double wristOffsetTicks = target.wristTarget - TerabytesIntoTheDeep.WRIST_ORIGIN;
        return (wristOffsetTicks / TerabytesIntoTheDeep.WRIST_RANGE) * TerabytesIntoTheDeep.WRIST_DEGREES_ALLOWABLE_HALF_RANGE;
    }

    public double getCurrentArmAngleDegrees() {
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;
        double armDegreesFromZero = averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE;
        return armDegreesFromZero - TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
    }

    public Double getCurrentEndEffectorHeight() {
        // Only return a height if we're in the collecting state.
        if (currentState != AppendageControlState.COLLECTING) {
            return null;
        }

        // Compute the average arm encoder ticks.
        int averageArmTicks = (currentArmLTicks + currentArmRTicks) / 2;

        // Compute the arm angle relative to horizontal (in degrees).
        // This uses the same conversion as in getCurrentArmAngleDegrees().
        double armAngleDegrees = (averageArmTicks / TerabytesIntoTheDeep.ARM_TICKS_PER_DEGREE)
                - TerabytesIntoTheDeep.ARM_LEVEL_DEGREES_ABOVE_ZERO;
        // Convert the angle to radians for trigonometric calculations.
        double armAngleRadians = Math.toRadians(armAngleDegrees);

        // Convert extender ticks to inches.
        // Note: In your setArmAndExtenderSetpoints method, the extension (in inches) is
        // applied above the minimum extension length.
        double extensionInches = currentExtenderTicks / TerabytesIntoTheDeep.EXTENDER_TICKS_PER_INCH;

        // The effective total distance from the pivot (arm axle) to the end effector.
        double effectiveLength = TerabytesIntoTheDeep.EXTENDER_MIN_LENGTH_INCHES + extensionInches;

        // The pivot (arm axle) is at a known height (ARM_AXLE_HEIGHT_INCHES).
        // The vertical displacement of the end effector from the pivot is given by the
        // effective length multiplied by the sine of the arm angle.
        double endEffectorHeight = TerabytesIntoTheDeep.ARM_AXLE_HEIGHT_INCHES + effectiveLength * Math.sin(armAngleRadians);

        return endEffectorHeight;
    }

    public boolean isWristMotionSettled() {
        // If we are not currently waiting for the wrist to settle, then it's settled.
        if (!waitingForWristSettle) {
            return true;
        }
        // If the elapsed time exceeds the dynamic timeout, consider it settled.
        if (wristSettleTimer.milliseconds() >= wristDynamicTimeoutMs) {
            waitingForWristSettle = false; // update the internal state for consistency
            return true;
        }
        return false;
    }
}
