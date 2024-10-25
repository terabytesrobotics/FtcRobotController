package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_YAW_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.BACK_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.FAST_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.FRONT_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.POSITION_ACQUIRED_INDICATE_MILLIS;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.POSITION_ACQUIRED_PULSE_MILLIS;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.SLOW_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_GAIN;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class TerabytesIntoTheDeep {

    // TODO: Trim these GPT fields down to only necessary
    private double fieldTheta;
    private double fieldRadius;
    private double squarenessParameter = 0.75;
    private ElapsedTime lastRadialModeLoopTime = new ElapsedTime();
    private static final double FIELD_SIDE_LENGTH = 60.0; // Not really real field side...it's the drivable square minus margins of robot
    private static final double MIN_FIELD_RADIUS = 46.0;
    private static final double MAX_FIELD_THETA_RATE = Math.toRadians(45);
    private static final double MAX_FIELD_RADIUS_RATE = 12.0;
    // TODO: Trim these GPT fields down to only necessary

    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getIntoTheDeepTagLibrary();
    private final SampleMecanumDrive drive;
    private final Gamepad gamepad1;
    private final WebcamName frontCamera;
    private final WebcamName backCamera;
    private final DigitalChannel indicator1Red;
    private final DigitalChannel indicator1Green;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private IntoTheDeepOpModeState state;
    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private Pose2d latestPoseEstimate = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private TerabytesCommand currentCommand = null;
    private final ArrayList<TerabytesCommand> commandSequence = new ArrayList<>();
    private IntoTheDeepOpModeState continuationState = null;
    private Pose2d lastAprilTagFieldPosition = null;
    private double lastAprilTagFieldPositionMillis = 0;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();
    private final boolean debugMode;
    private final AllianceColor allianceColor;
    private final Servo pincer;

    private final OnActivatedEvaluator rb1ActivatedEvaluator;
    private final OnActivatedEvaluator a1ActivatedEvaluator;
    private final OnActivatedEvaluator b1ActivatedEvaluator;
    private final OnActivatedEvaluator y1ActivatedEvaluator;
    private final OnActivatedEvaluator x1ActivatedEvaluator;

    public TerabytesIntoTheDeep(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.state = IntoTheDeepOpModeState.HEADLESS_DRIVE;
        this.debugMode = debugMode;

        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        SwitchableCameraName switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .build();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        indicator1Red = hardwareMap.get(DigitalChannel.class, "indicator1red");
        indicator1Green = hardwareMap.get(DigitalChannel.class, "indicator1green");
        indicator1Red.setMode(DigitalChannel.Mode.OUTPUT);
        indicator1Green.setMode(DigitalChannel.Mode.OUTPUT);

        rb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        a1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
        y1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.y);
        x1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.x);

        pincer = hardwareMap.get(Servo.class, "pincer");
    }

    public Pose2d getLatestPoseEstimate() {
        return latestPoseEstimate;
    }

    public TelemetryPacket getTelemetryPacket() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", latestPoseEstimate.getX());
        packet.put("y", latestPoseEstimate.getY());
        packet.put("heading", latestPoseEstimate.getHeading());

        Double[] motorCurrents = drive.getMotorCurrents();
        Double leftFrontCurrent = motorCurrents[0];
        Double leftRearCurrent = motorCurrents[1];
        Double rightFrontCurrent = motorCurrents[2];
        Double rightRearCurrent = motorCurrents[3];

        Double[] motorPowers = drive.getMotorPowers();
        Double leftFrontPower = motorPowers[0];
        Double leftRearPower = motorPowers[1];
        Double rightFrontPower = motorPowers[2];
        Double rightRearPower = motorPowers[3];

        Double[] motorVelocities = drive.getMotorVelocities();
        Double leftFrontVelocity = motorVelocities[0];
        Double leftRearVelocity = motorVelocities[1];
        Double rightFrontVelocity = motorVelocities[2];
        Double rightRearVelocity = motorVelocities[3];


        Pose2d poseVelocity = drive.getPoseVelocity();

        packet.put("lfc", leftFrontCurrent);
        packet.put("lrc", leftRearCurrent);
        packet.put("rfc", rightFrontCurrent);
        packet.put("rrc", rightRearCurrent);

        packet.put("lfp", leftFrontPower);
        packet.put("lrp", leftRearPower);
        packet.put("rfp", rightFrontPower);
        packet.put("rrp", rightRearPower);

        packet.put("lfv", leftFrontVelocity);
        packet.put("lrv", leftRearVelocity);
        packet.put("rfv", rightFrontVelocity);
        packet.put("rrv", rightRearVelocity);

        return packet;
    }

    public void autonomousInit(Pose2d startPose, TerabytesAutonomousPlan autonomousPlan) {
        drive.setPoseEstimate(startPose);
        activateFrontCameraProcessing();
        setCommandSequence(autonomousPlan.getCommandSequence(allianceColor));
    }

    public void teleopInit(Pose2d startPose) {
        drive.setPoseEstimate(startPose == null ? new Pose2d() : startPose);
        activateFrontCameraProcessing();
    }

    public void startup(IntoTheDeepOpModeState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
    }

    private void activateFrontCameraProcessing() {
        visionPortal.setActiveCamera(frontCamera);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    // Control loop.  Returns true iff the op-mode should continue running.
    public boolean evaluate() {
        // Do the things we should do regardless of state
        // keep updating the drive and keep the machine alive
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        evaluateIndicatorLights();
        evaluatePincer();

        // Determine whether there's been a state change
        // Run the state specific control logic
        IntoTheDeepOpModeState currentState = state;
        IntoTheDeepOpModeState nextState = evaluatePositioningSystems(currentState);

        switch (state) {
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence();
                break;
            case HEADLESS_DRIVE:
                nextState = evaluateHeadlessDrivingMode();
                break;
            case RADIAL_DRIVING_MODE:
                nextState = evaluateRadialDrivingMode();
                break;
            case SUBMERSIBLE_APPROACH:
                nextState = evaluateSubmersibleApproach();
                break;
            case BASKET_APPROACH:
                nextState = evaluateBasketApproach();
                break;
            case STOPPED_UNTIL_END:
                // Hold the drive still.
                setDrivePower(new Pose2d());
            default:
                break;
        }

        // Reset the timer so that the state logic can use it to tell how long it's been in that state
        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        return state != IntoTheDeepOpModeState.HALT_OPMODE;
    }

    private void evaluatePincer() {
        if (gamepad1.left_bumper) {
            pincer.setPosition(0.7);
        } else {
            pincer.setPosition(0.4);
        }
    }

    private IntoTheDeepOpModeState evaluateSubmersibleApproach() {
        if (latestPoseEstimate == null) {
            // Cannot proceed without pose estimate
            return IntoTheDeepOpModeState.HEADLESS_DRIVE;
        }

        double x = latestPoseEstimate.getX();
        double y = latestPoseEstimate.getY();
        double robotFieldTheta = Math.atan2(y, x);

        // Define a helper class for approach targets
        class ApproachTarget {
            public Pose2d pose;
            public double fieldTheta;

            public ApproachTarget(Pose2d pose) {
                this.pose = pose;
                this.fieldTheta = Math.atan2(pose.getY(), pose.getX());
            }
        }

        // Create an array of possible approach targets
        ApproachTarget[] approachTargets = new ApproachTarget[] {
                new ApproachTarget(IntoTheDeepPose.SUBMERSIBLE_APPROACH_ALLIANCE_SIDE.getPose(allianceColor)),
                new ApproachTarget(IntoTheDeepPose.SUBMERSIBLE_APPROACH_REAR_SIDE.getPose(allianceColor)),
                new ApproachTarget(IntoTheDeepPose.SUBMERSIBLE_APPROACH_OPPONENT_SIDE.getPose(allianceColor)),
                new ApproachTarget(IntoTheDeepPose.SUBMERSIBLE_APPROACH_AUDIENCE_SIDE.getPose(allianceColor))
        };

        // Select the closest target based on angular difference
        ApproachTarget selectedTarget = null;
        double minDistance = Double.MAX_VALUE;
        for (ApproachTarget approachTarget : approachTargets) {
            Pose2d approachPose = approachTarget.pose;
            double deltaX = approachPose.getX() - x;
            double deltaY = approachPose.getY() - y;
            double distance = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
            if (distance < minDistance) {
                minDistance = distance;
                selectedTarget = approachTarget;
            }
        }

        // Command the robot to approach the selected target
        if (selectedTarget != null) {
            setDrivePower(getPoseTargetAutoDriveControl(selectedTarget.pose));
        }

        // Check for exit condition: if 'x' button is pressed, return to radial driving mode
        if (x1ActivatedEvaluator.evaluate()) {
            initializeRadialDrivingMode();
            return IntoTheDeepOpModeState.RADIAL_DRIVING_MODE;
        }

        return IntoTheDeepOpModeState.SUBMERSIBLE_APPROACH;
    }

    private IntoTheDeepOpModeState evaluateBasketApproach() {
        if (latestPoseEstimate == null) {
            // Cannot proceed without pose estimate
            return IntoTheDeepOpModeState.HEADLESS_DRIVE;
        }

        Pose2d basketApproach = IntoTheDeepPose.BASKET_APPROACH.getPose(allianceColor);
        setDrivePower(getPoseTargetAutoDriveControl(basketApproach));

        if (x1ActivatedEvaluator.evaluate()) {
            initializeRadialDrivingMode();
            return IntoTheDeepOpModeState.RADIAL_DRIVING_MODE;
        }

        return IntoTheDeepOpModeState.BASKET_APPROACH;
    }

    private IntoTheDeepOpModeState evaluateHeadlessDrivingMode() {
        if (rb1ActivatedEvaluator.evaluate() && hasPositionEstimate()) {
            initializeRadialDrivingMode();
            return IntoTheDeepOpModeState.RADIAL_DRIVING_MODE;
        }

        boolean slowMode = gamepad1.left_bumper;

        Pose2d driveInput = getScaledHeadlessDriverInput(gamepad1, allianceColor.OperatorHeadingOffset, slowMode);
        setDrivePower(driveInput);

        return IntoTheDeepOpModeState.HEADLESS_DRIVE;
    }


    private IntoTheDeepOpModeState evaluateCommandSequence() {
        if (commandSequence.isEmpty()) {
            IntoTheDeepOpModeState _continuationState = continuationState;
            continuationState = null;
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            return _continuationState == null ? IntoTheDeepOpModeState.STOPPED_UNTIL_END : _continuationState;
        }

        if (currentCommand == null) {
            currentCommand = commandSequence.get(0);
            currentCommandTime.reset();
            currentCommandSettledTime.reset();

            if (currentCommand.FrontCameraOn != null && currentCommand.FrontCameraOn) {
                activateFrontCameraProcessing();
            }
        }

        if (currentCommand.DriveDirectToPose != null) {
            setDrivePower(
                    getPoseTargetAutoDriveControl(currentCommand.DriveDirectToPose));
        }

        boolean driveCompleted = currentCommand.DriveDirectToPose == null || isAtPoseTarget(currentCommand.DriveDirectToPose, currentCommand.SettleThresholdRatio);
        boolean settledRightNow = driveCompleted;

        boolean minTimeElapsed = currentCommandTime.milliseconds() > currentCommand.MinTimeMillis;
        boolean commandCompleted = settledRightNow && minTimeElapsed && currentCommandSettledTime.milliseconds() > currentCommand.SettleTimeMillis;
        boolean debugAdvance = !debugMode || gamepad1.a;
        if (commandCompleted && debugAdvance) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            setDrivePower(new Pose2d());
            commandSequence.remove(0);
            currentCommand = null;
        } else if (!settledRightNow) {
            currentCommandSettledTime.reset();
        }

        return IntoTheDeepOpModeState.COMMAND_SEQUENCE;
    }

    private void evaluateIndicatorLights() {

        double timeSincePositionSet = (timeSinceStart.milliseconds() - lastAprilTagFieldPositionMillis);
        if (lastAprilTagFieldPosition != null && timeSincePositionSet < POSITION_ACQUIRED_INDICATE_MILLIS) {
            if (timeSincePositionSet % (2 * POSITION_ACQUIRED_PULSE_MILLIS) > POSITION_ACQUIRED_PULSE_MILLIS) {
                indicator1Green.setState(true);
                indicator1Red.setState(true);
            } else {
                indicator1Green.setState(false);
                indicator1Red.setState(false);
            }
        }

        if (hasPositionEstimate()) {
            indicator1Green.setState(true);
            indicator1Red.setState(false);
        } else {
            indicator1Green.setState(false);
            indicator1Red.setState(false);
        }
    }

    private IntoTheDeepOpModeState evaluatePositioningSystems(IntoTheDeepOpModeState currentState) {
        boolean usingBackCamera = visionPortal.getActiveCamera().equals(backCamera);
        double cameraDistanceOffset = usingBackCamera ? BACK_CAMERA_OFFSET_INCHES : FRONT_CAMERA_OFFSET_INCHES;
        double cameraAngleOffset = usingBackCamera ? Math.PI : 0;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                // This sometimes happens.
                if (detection.ftcPose == null) {
                    continue;
                }

                if (detection.ftcPose.range > APRIL_TAG_RECOGNITION_MAX_RANGE ||
                        detection.ftcPose.range < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                        Math.abs(Math.toRadians(detection.ftcPose.bearing)) > APRIL_TAG_RECOGNITION_BEARING_THRESHOLD ||
                        Math.abs(Math.toRadians(detection.ftcPose.yaw)) > APRIL_TAG_RECOGNITION_YAW_THRESHOLD) {
                    continue;
                }

                Pose2d estimatedPose = calculateRobotPose(detection, cameraDistanceOffset, cameraAngleOffset);
                if (poseQueue.size() >= APRIL_TAG_QUEUE_CAPACITY) {
                    poseQueue.poll(); // Remove the oldest element
                }
                poseQueue.offer(estimatedPose);
            }
        }

        // Calculate average and variance
        if (poseQueue.size() == APRIL_TAG_QUEUE_CAPACITY) {
            Pose2d averagePose = calculateAveragePose(poseQueue);
            Pose2d variancePose = calculateVariancePose(poseQueue, averagePose);

            // Thresholds for variance, adjust as needed
            double translationVarianceThreshold = 1.0;
            double headingVarianceThreshold = Math.PI / 32;
            if (variancePose.getX() <= translationVarianceThreshold && variancePose.getY() <= translationVarianceThreshold && variancePose.getHeading() <= headingVarianceThreshold) {
                drive.setPoseEstimate(averagePose);
                lastAprilTagFieldPosition = averagePose;
                lastAprilTagFieldPositionMillis = timeSinceStart.milliseconds();
                poseQueue.clear();
            }
        }

        return currentState;
    }

    private Pose2d calculateAveragePose(Queue<Pose2d> poses) {
        double sumX = 0, sumY = 0, sumHeading = 0;
        for (Pose2d pose : poses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumHeading += pose.getHeading();
        }
        int count = poses.size();
        return new Pose2d(sumX / count, sumY / count, Angle.norm(sumHeading / count));
    }

    private Pose2d calculateVariancePose(Queue<Pose2d> poses, Pose2d averagePose) {
        double varianceX = 0, varianceY = 0, varianceHeading = 0;
        for (Pose2d pose : poses) {
            varianceX += Math.pow(pose.getX() - averagePose.getX(), 2);
            varianceY += Math.pow(pose.getY() - averagePose.getY(), 2);
            varianceHeading += Math.pow(Angle.normDelta(pose.getHeading() - averagePose.getHeading()), 2);
        }
        int count = poses.size();
        return new Pose2d(Math.sqrt(varianceX / count), Math.sqrt(varianceY / count), Math.sqrt(varianceHeading / count));
    }

    private boolean isAtPoseTarget(Pose2d target, double thresholdRatio) {
        Pose2d error = getPoseTargetError(target);
        if (latestPoseEstimate == null || error == null) return false;
        return Math.hypot(error.getX(), error.getY()) < (DRIVE_TO_POSE_THRESHOLD * thresholdRatio) &&
                Math.abs(error.getHeading()) < (TURN_ERROR_THRESHOLD * thresholdRatio);
    }

    private Pose2d getPoseTargetError(Pose2d poseTarget) {
        if (latestPoseEstimate == null) return null;
        return new Pose2d(poseTarget.getX() - latestPoseEstimate.getX(),
                poseTarget.getY() - latestPoseEstimate.getY(),
                Angle.normDelta(poseTarget.getHeading() - latestPoseEstimate.getHeading()));
    }

    private Pose2d getPoseTargetAutoDriveControl(Pose2d poseTarget) {
        Pose2d error = getPoseTargetError(poseTarget);
        if (latestPoseEstimate == null || error == null) return new Pose2d();

        double robotHeading = latestPoseEstimate.getHeading();

        double translationErrorMagnitude = Math.hypot(error.getX(), error.getY());
        double translationErrorFieldHeading = Math.atan2(error.getY(), error.getX());

        double adjustedHeading = translationErrorFieldHeading - robotHeading;
        double robotXErrorMagnitude = translationErrorMagnitude * Math.cos(adjustedHeading);
        double robotYErrorMagnitude = translationErrorMagnitude * Math.sin(adjustedHeading);

        return new Pose2d(
                Range.clip(robotXErrorMagnitude * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                Range.clip(robotYErrorMagnitude * SPEED_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                Range.clip(error.getHeading() * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection, double cameraRobotOffset, double cameraRobotHeadingOffset) {
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(detection.id);
        if (tag == null) return null;

        double yaw = Math.toRadians(detection.ftcPose.yaw);
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double range = detection.ftcPose.range;

        // Get the correct tagFieldHeading based on the tag ID
        double tagFieldHeading = getTagFieldHeading(detection.id);

        double tagToCameraHeading = Angle.norm(tagFieldHeading + bearing - yaw);
        double cameraFieldX = tag.fieldPosition.get(0) + (range * Math.cos(tagToCameraHeading));
        double cameraFieldY = tag.fieldPosition.get(1) + (range * Math.sin(tagToCameraHeading));
        double cameraFieldHeading = Angle.norm(
                tagFieldHeading + Math.PI + cameraRobotHeadingOffset - yaw);

        double robotFieldX = cameraFieldX - (cameraRobotOffset * Math.cos(cameraFieldHeading));
        double robotFieldY = cameraFieldY - (cameraRobotOffset * Math.sin(cameraFieldHeading));

        return new Pose2d(robotFieldX, robotFieldY, cameraFieldHeading);
    }

    private double getTagFieldHeading(int tagId) {
        switch (tagId) {
            case 11: // BlueAudienceWall
            case 16: // RedAudienceWall
                return 0;
            case 12: // BlueAllianceWall
                return -Math.PI / 2;
            case 13: // BlueRearWall
            case 14: // RedRearWall
                return Math.PI;
            case 15: // RedAllianceWall
                return Math.PI / 2;
            default:
                return 0; // Default to 0 if unknown
        }
    }

    private void setCommandSequence(List<TerabytesCommand> commands) {
        setCommandSequence(IntoTheDeepOpModeState.STOPPED_UNTIL_END, commands);
    }

    private void setCommandSequence(IntoTheDeepOpModeState _continuationState, List<TerabytesCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        continuationState = _continuationState;
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad, double operatorHeadingOffset, boolean slow) {
        Vector2d inputFieldDirection = TerabytesHelpers.headlessLeftStickFieldDirection(gamepad, operatorHeadingOffset, latestPoseEstimate.getHeading());
        double scale = slow ? SLOW_MODE_SCALE : FAST_MODE_SCALE;
        double scaledRobotX = inputFieldDirection.getX() * scale;
        double scaledRobotY = inputFieldDirection.getY() * scale;
        double scaledRotation = -gamepad.right_stick_x * scale;
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    public void shutDown() {
        setDrivePower(new Pose2d());
        visionPortal.close();
    }

    public void setDrivePower(Pose2d drivePower) {
        Pose2d normalized = drivePower;
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            double denom = Math.abs(drivePower.getX())
                    + Math.abs(drivePower.getY())
                    + Math.abs(drivePower.getHeading());

            normalized = new Pose2d(
                    drivePower.getX(),
                    drivePower.getY(),
                    drivePower.getHeading()
            ).div(denom);
        }

        // This should be the only place we call drive.setDrivePower
        drive.setDrivePower(new Pose2d(
                Math.abs(normalized.getX()) < 0.005 ? 0 : normalized.getX(),
                Math.abs(normalized.getY()) < 0.005 ? 0 : normalized.getY(),
                Math.abs(normalized.getHeading()) < 0.005 * Math.PI ? 0 : normalized.getHeading()
        ));
    }

    private void initializeRadialDrivingMode() {
        Pose2d currentPose = latestPoseEstimate;
        double x = currentPose.getX();
        double y = currentPose.getY();

        fieldTheta = Math.atan2(y, x);

        // Compute the maximum allowable radius at the current theta
        double maxRadius = fieldRadiusAtTheta(fieldTheta);

        // Set fieldRadius to the lesser of the current radius and max allowable radius
        fieldRadius = Math.min(Math.hypot(x, y), maxRadius);

        lastRadialModeLoopTime.reset();
    }

    private IntoTheDeepOpModeState evaluateRadialDrivingMode() {
        // Compute deltaTime
        double deltaTime = lastRadialModeLoopTime.seconds();
        lastRadialModeLoopTime.reset();

        // Adjust fieldTheta and fieldRadius based on joystick inputs
        double deltaTheta = -gamepad1.right_stick_y * MAX_FIELD_THETA_RATE * deltaTime;
        double deltaRadius = -gamepad1.left_stick_y * MAX_FIELD_RADIUS_RATE * deltaTime;

        fieldTheta += deltaTheta;
        fieldTheta = Angle.norm(fieldTheta); // Keep between -π and π

        fieldRadius += deltaRadius;

        // Limit fieldRadius to be between minRadius and maxRadius
        double minRadius = MIN_FIELD_RADIUS;
        double maxRadius = fieldRadiusAtTheta(fieldTheta);
        fieldRadius = Range.clip(fieldRadius, minRadius, maxRadius);

        // Compute target pose
        double xTarget = fieldRadius * Math.cos(fieldTheta);
        double yTarget = fieldRadius * Math.sin(fieldTheta);
        double headingTarget = fieldTheta + (Math.PI / 2); // Face away from (0,0)

        Pose2d targetPose = new Pose2d(xTarget, yTarget, headingTarget);

        // Command robot to drive towards target pose
        setDrivePower(getPoseTargetAutoDriveControl(targetPose));

        // Exit condition to return to manual control
        if (rb1ActivatedEvaluator.evaluate()) {
            return IntoTheDeepOpModeState.HEADLESS_DRIVE;
        } else if (a1ActivatedEvaluator.evaluate()) {
            return IntoTheDeepOpModeState.SUBMERSIBLE_APPROACH;
        } else if (y1ActivatedEvaluator.evaluate()) {
            return IntoTheDeepOpModeState.BASKET_APPROACH;
        }

        return IntoTheDeepOpModeState.RADIAL_DRIVING_MODE;
    }

    private double fieldRadiusAtTheta(double theta) {
        double s = FIELD_SIDE_LENGTH; // Side length
        double k = 2 + (squarenessParameter * 48); // k from 2 (circle) to 50 (square)

        double denom = Math.pow(Math.abs(Math.cos(theta)), k) + Math.pow(Math.abs(Math.sin(theta)), k);
        double radius = s / Math.pow(denom, 1.0 / k);

        return radius;
    }
}
