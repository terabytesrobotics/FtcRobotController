package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_YAW_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.Constants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TURN_GAIN;

import android.util.ArrayMap;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.teamcode.Processors.SampleDetectVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

public class DecodeRobotControl {






    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

    // Basic state
    private final boolean debugMode;
    private boolean isAutonomous = false;
    private OpModeState state;
    private final ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime timeSinceInit = new ElapsedTime();
    private ElapsedTime timeSinceStart = new ElapsedTime();
    private ElapsedTime timeInState = new ElapsedTime();
    private Pose2d latestPoseEstimate = null;
    private int armLTicksAtInit = 0;
    private int armRTicksAtInit = 0;
    private int extenderTicksAtInit = 0;
    private final AllianceColor allianceColor;
    private Pose2d lastAprilTagFieldPosition = null;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();
    private final ArrayList<OpModeCommand> commandSequence = new ArrayList<>();
    private OpModeCommand currentCommand = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private OpModeState continuationState = null;
    private final SampleMecanumDrive drive;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final OnActivatedEvaluator rb1ActivatedEvaluator;
    private final OnActivatedEvaluator lb1ActivatedEvaluator;
    private final OnActivatedEvaluator a1ActivatedEvaluator;
    private final OnActivatedEvaluator b1ActivatedEvaluator;
    private final OnActivatedEvaluator y1ActivatedEvaluator;
    private final OnActivatedEvaluator x1ActivatedEvaluator;
    private final OnActivatedEvaluator a2ActivatedEvaluator;
    private final OnActivatedEvaluator rb2ActivatedEvaluator;
    private final OnActivatedEvaluator x2ActivatedEvaluator;
    private final OnActivatedEvaluator y2ActivatedEvaluator;
    private final OnActivatedEvaluator dpu1ActivatedEvaluator;
    private final OnActivatedEvaluator dpd1ActivatedEvaluator;
    private final Servo servo1;

    private final AprilTagProcessor aprilTagProcessor;

    private final SampleDetectVisionProcessor sampleDetectVisionProcessor;



    public DecodeRobotControl(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.state = OpModeState.MANUAL_CONTROL;
        this.debugMode = debugMode;

        servo1 = hardwareMap.get(Servo.class, "Servo 1");

        aprilTagProcessor = new AprilTagProcessor.Builder().build();


        EnumSet<SampleDetectVisionProcessor.DetectableColor> colorsToDetect;
        switch (allianceColor) {
            case RED:
                colorsToDetect = EnumSet.of(SampleDetectVisionProcessor.DetectableColor.RED,
                        SampleDetectVisionProcessor.DetectableColor.YELLOW);
                break;
            case BLUE:
                colorsToDetect = EnumSet.of(SampleDetectVisionProcessor.DetectableColor.BLUE,
                        SampleDetectVisionProcessor.DetectableColor.YELLOW);
                break;
            default:
                colorsToDetect = EnumSet.of(
                    SampleDetectVisionProcessor.DetectableColor.RED,
                    SampleDetectVisionProcessor.DetectableColor.BLUE,
                    SampleDetectVisionProcessor.DetectableColor.YELLOW);
                break;
        }

        sampleDetectVisionProcessor = new SampleDetectVisionProcessor(colorsToDetect);



        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        a1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
        y1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.y);
        x1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.x);
        rb2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);
        a2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        y2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        x2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        dpu1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.dpad_up);
        dpd1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.dpad_down);
        lb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);
    }

    private Map<String, String> logData = new ArrayMap<>();
    public Map<String, String> getLogData() {
        logData.clear();

        return logData;
    }

    private Pose2d driveInput = new Pose2d();
    public TelemetryPacket getTelemetryPacket() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("loopTime", loopTime.milliseconds());
        packet.put("x", latestPoseEstimate.getX());
        packet.put("y", latestPoseEstimate.getY());
        packet.put("heading", latestPoseEstimate.getHeading());

        // TODO: Get this reported into telemetry
        packet.put("currentState", state.toString());
        if (lastAprilTagFieldPosition != null) {
            packet.put("estimate-x", lastAprilTagFieldPosition.getX());
            packet.put("estimate-y", lastAprilTagFieldPosition.getY());
            packet.put("etimate-heading", lastAprilTagFieldPosition.getHeading());
        }

        packet.put("armLTicksAtInit", armLTicksAtInit);
        packet.put("armRTicksAtInit", armRTicksAtInit);
        packet.put("extenderTicksAtInit", extenderTicksAtInit);

        packet.put("DriveInputX", driveInput.getX());
        packet.put("DriveInputY", driveInput.getY());
        packet.put("VisionProcessorAngle", sampleDetectVisionProcessor.detectedEllipseAngle);
        packet.put("VisionXError", sampleDetectVisionProcessor.detectedExtenderErrorSignal);
        packet.put("VisionYErroqr", sampleDetectVisionProcessor.detectedLateralErrorSignal);
        return packet;
    }

    public void autonomousInit(AutonomousPlan autonomousPlan) {
        timeSinceInit.reset();
        isAutonomous = true;
        drive.setPoseEstimate(new Pose2d());
        setCommandSequence(new ArrayList());
    }

    public void teleopInit(Pose2d startPose) {
        timeSinceInit.reset();
        drive.setPoseEstimate(startPose);
        lastAprilTagFieldPosition = startPose;
    }

    public void initializeMechanicalBlocking() {
        state = OpModeState.MANUAL_CONTROL;
    }

    public void startup(OpModeState startupState) {
        timeSinceStart.reset();
        timeInState.reset();
        state = startupState;
    }

    private void evaluateSwitchCamera() {
        // One camera only
        //visionPortal.setActiveCamera(frontCamera);
        //visionPortal.setProcessorEnabled(sampleDetectVisionProcessor, false);
        //visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public boolean evaluate() {
        double dt = loopTime.milliseconds();
        loopTime.reset();
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        evaluateSwitchCamera();
        evaluatePositioningSystems();

        boolean debugKill = debugMode &&
                ((gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a) ||
                        (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.a));

        OpModeState currentState = state;
        OpModeState nextState = currentState;
        switch (currentState) {
            case MANUAL_CONTROL:
                nextState = evaluateManualControl(dt);
                break;
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence();
                break;
            case STOPPED_UNTIL_END:
                setDrivePower(new Pose2d());
                break;
            default:
                break;
        }

        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        return state != OpModeState.HALT_OPMODE;
    }

    private double dist(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private OpModeState evaluateManualControl(double dtMillis) {
        servo1.setPosition(gamepad1.right_stick_x);

        boolean fastMode = gamepad1.left_bumper;
        boolean hasPositionEstimate = hasPositionEstimate();

        driveInput = driveInput
                .plus(getScaledHeadlessDriverInput(gamepad1, allianceColor.OperatorHeadingOffset))
                .plus(getScaledHeadlessDriverABInput(gamepad1, allianceColor.OperatorHeadingOffset));


        if (fastMode) {
            driveInput = driveInput.div(1.5);
        } else {
            driveInput = driveInput.div(2.25);
        }

        setDrivePower(driveInput);
        return OpModeState.MANUAL_CONTROL;
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad, double operatorHeadingOffset) {
        Vector2d inputFieldDirection = Helpers.headlessLeftStickFieldDirection(gamepad, operatorHeadingOffset, latestPoseEstimate.getHeading());
        double scaledRobotX = inputFieldDirection.getX();
        double scaledRobotY = inputFieldDirection.getY();
        double signRotation = -Math.signum(gamepad.right_stick_x);
        double scaledRotation = signRotation * (gamepad.right_stick_x * gamepad.right_stick_x);
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private Pose2d getScaledHeadlessDriverABInput(Gamepad gamepad, double operatorHeadingOffset) {
        Vector2d inputFieldDirection = Helpers.headlessABButtonFieldDirection(gamepad, operatorHeadingOffset, latestPoseEstimate.getHeading());
        double scaledRobotX = inputFieldDirection.getX();
        double scaledRobotY = inputFieldDirection.getY();
        double scaledRotation = -gamepad.right_stick_x;
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private OpModeState evaluateCommandSequence() {
        if (commandSequence.isEmpty()) {
            OpModeState _continuationState = continuationState;
            continuationState = null;
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            return _continuationState == null ? OpModeState.STOPPED_UNTIL_END : _continuationState;
        }

        if (currentCommand == null) {
            currentCommand = commandSequence.get(0);
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
        }

        if (timeSinceStart.milliseconds() < currentCommand.WaitUntilElapsedMillis) {
            setDrivePower(new Pose2d());
            return OpModeState.COMMAND_SEQUENCE;
        }

        if (currentCommand.DriveToPose != null) {
            setDrivePower(
                    getPoseTargetAutoDriveControl(currentCommand.DriveToPose));
        }

        boolean driveCompleted = currentCommand.DriveToPose == null || isAtPoseTarget(currentCommand.DriveToPose, currentCommand.DriveSettleThresholdRatio);
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

        return OpModeState.COMMAND_SEQUENCE;
    }

    private void evaluatePositioningSystems() {
        double cameraDistanceOffset = FRONT_CAMERA_OFFSET_INCHES;
        double cameraAngleOffset = 0;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
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
                    poseQueue.poll();
                }
                poseQueue.offer(estimatedPose);
            }
        }

        if (poseQueue.size() == APRIL_TAG_QUEUE_CAPACITY) {
            Pose2d averagePose = calculateAveragePose(poseQueue);
            Pose2d variancePose = calculateVariancePose(poseQueue, averagePose);

            double translationVarianceThreshold = 2.0;
            double headingVarianceThreshold = Math.PI / 16;
            if (variancePose.getX() <= translationVarianceThreshold &&
                    variancePose.getY() <= translationVarianceThreshold &&
                    variancePose.getHeading() <= headingVarianceThreshold &&
                    !isAutonomous) {
                drive.setPoseEstimate(averagePose);
                lastAprilTagFieldPosition = averagePose;
                poseQueue.clear();
            }
        }
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

        double distance = Math.hypot(error.getX(), error.getY());
        double robotHeading = latestPoseEstimate.getHeading();
        double headingToError = Math.atan2(error.getY(), error.getX()) - robotHeading;
        double xErr = distance * Math.cos(headingToError);
        double yErr = distance * Math.sin(headingToError);
        boolean xErrEliminated = Math.abs(xErr) < 0.75;
        boolean yErrEliminated = Math.abs(yErr) < 0.75;
        boolean thetaErrEliminated = Math.abs(error.getHeading()) < (Math.PI / 15);

        double minPower = 0.24530625;
        double minRotation = 0.63;
        double xMin = xErrEliminated ? 0 : Math.signum(xErr) * minPower;
        double yMin = yErrEliminated ? 0 : Math.signum(yErr) * minPower;
        double thetaMin = thetaErrEliminated ? 0 : Math.signum(error.getHeading()) * minRotation;

        double x = Math.abs(minPower) > Math.abs(xErr * SPEED_GAIN) ? xMin : xErr * SPEED_GAIN;
        double y = Math.abs(minPower) > Math.abs(yErr * SPEED_GAIN) ? yMin : yErr * SPEED_GAIN;
        double theta = Math.abs(thetaMin) > Math.abs(error.getHeading() * TURN_GAIN) ? thetaMin : error.getHeading() * TURN_GAIN;

        double xPower = Range.clip(x, -1, 1);
        double yPower = Range.clip(y, -1, 1);
        double hPower = Range.clip(theta, -1, 1);

        return new Pose2d(xPower, yPower, hPower);
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection, double cameraRobotOffset, double cameraRobotHeadingOffset) {
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(detection.id);
        if (tag == null) return null;

        double yaw = Math.toRadians(detection.ftcPose.yaw);
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double range = detection.ftcPose.range;

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
            case 11:
            case 16:
                return 0;
            case 12:
                return -Math.PI / 2;
            case 13:
            case 14:
                return Math.PI;
            case 15:
                return Math.PI / 2;
            default:
                return 0;
        }
    }

    private void setCommandSequence(List<OpModeCommand> commands) {
        setCommandSequence(OpModeState.STOPPED_UNTIL_END, commands);
    }

    private void setCommandSequence(OpModeState _continuationState, List<OpModeCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        continuationState = _continuationState;
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    public void shutDown() {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrivePower(new Pose2d());
        //visionPortal.close();
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

        drive.setDrivePower(new Pose2d(
                Math.abs(normalized.getX()) < 0.005 ? 0 : normalized.getX(),
                Math.abs(normalized.getY()) < 0.005 ? 0 : normalized.getY(),
                Math.abs(normalized.getHeading()) < 0.005 * Math.PI ? 0 : normalized.getHeading()
        ));
    }
}
