package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NibusConstants.*;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.teamcode.util.TrueForTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class Nibus2000 {

    private final SampleMecanumDrive drive;
//    private final TouchSensor armMin;
//    private final TouchSensor extenderMin;
    private final PIDController armcontrol;

    public double extendLengthCm = 0;
    public int extendTicTarget = 0;
    public double armTargetDegrees = 0;
    public double target = 0.0;
    //private final DcMotorEx arm_motor0;
    //private final DcMotorEx extender;
    //private final DcMotorEx launcher;
//    Servo greenGrabber;
//    Servo blueGrabber;
//    Servo wrist;
//    Servo launcherWrist;
    private CollectorState collectorState = CollectorState.DRIVING_SAFE;
    private final OnActivatedEvaluator a1PressedEvaluator;
    private final OnActivatedEvaluator x1PressedEvaluator;
    private final OnActivatedEvaluator y1PressedEvaluator;
    private final OnActivatedEvaluator b1PressedEvaluator;
    private final OnActivatedEvaluator lb1PressedEvaluator;
    private final OnActivatedEvaluator rb1PressedEvaluator;
    private final OnActivatedEvaluator a2PressedEvaluator;
    private final OnActivatedEvaluator b2PressedEvaluator;
    private final OnActivatedEvaluator x2PressedEvaluator;
    private final OnActivatedEvaluator y2PressedEvaluator;
    private final OnActivatedEvaluator lb2PressedEvaluator;
    private final OnActivatedEvaluator rb2PressedEvaluator;
    private final OnActivatedEvaluator dpadUp2PressedEvaluator;
    private final OnActivatedEvaluator dpadDown2PressedEvaluator;
    private final OnActivatedEvaluator dpadLeft2PressedEvaluator;
    private final OnActivatedEvaluator dpadRight2PressedEvaluator;
    private final TrueForTime settledAtPoseTarget;
    private final OnActivatedEvaluator rs1PressedEvaluator;
    private final OnActivatedEvaluator ls1PressedEvaluator;

    private final OnActivatedEvaluator onEnteredUpstageEvaluator;
    private final OnActivatedEvaluator onEnteredBackstageEvaluator;
    private BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
    private GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final WebcamName frontCamera;
    private final WebcamName backCamera;
    private final SwitchableCameraName switchableCamera;
    private AllianceColor allianceColor;
    private AlliancePose alliancePose;
    private NibusState state;
    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private WindowBoxesVisionProcessor propFinder;
    private Pose2d latestPoseEstimate = null;
    private int currentArmPosition = 0;
    private int currentExtenderPosition = 0;
    private int armTrimIncrements = 0;
    private int wristTrimIncrements = 0;
    private boolean launchingAirplane = false;
    private int launchingAirplaneTimeMillis = 0;
    private int endgameLiftStage = 0;
    private NibusAutonomousPlan autonomousPlan = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private NibusAutonomousCommand currentCommand = null;
    private final ArrayList<NibusAutonomousCommand> commandSequence = new ArrayList<>();
    private NibusState continuationState = null;
    private int framesProcessed = 0;
    private final int[] leftMidRightVotes = new int[] { 0, 0, 0 };
    private AlliancePropPosition alliancePropPosition = null;
    private final ElapsedTime elapsedPropFrameTime = new ElapsedTime();
    private DigitalChannel indicator1Red;
    private DigitalChannel indicator1Green;
    private RevBlinkinLedDriver blinkinLedDriver;
    private NibusApproach nextNibusApproach = null;
    private ElapsedTime approachSettlingTimer = null;
    private boolean hasAprilTagFieldPosition = false;
    private Pose2d poseTarget = null;
    private NibusApproach approachTarget = null;
    private Queue<Pose2d> poseQueue = new LinkedList<>();
    private static final int QUEUE_CAPACITY = 12;

    public Nibus2000(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.state = NibusState.MANUAL_DRIVE;

        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        propFinder = new WindowBoxesVisionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(propFinder)
                .addProcessor(aprilTagProcessor)
                .build();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        indicator1Red = hardwareMap.get(DigitalChannel.class, "indicator1red");
        indicator1Green = hardwareMap.get(DigitalChannel.class, "indicator1green");
        indicator1Red.setMode(DigitalChannel.Mode.OUTPUT);
        indicator1Green.setMode(DigitalChannel.Mode.OUTPUT);

        settledAtPoseTarget = new TrueForTime(APPROACH_SETTLE_TIME_MS, () -> {
            Pose2d error = getPoseTargetError(poseTarget);
            if (latestPoseEstimate == null || error == null) return false;
            return Math.hypot(error.getX(), error.getY()) < DRIVE_TO_POSE_THRESHOLD &&
                    Math.abs(error.getHeading()) < TURN_ERROR_THRESHOLD;
        });
        a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
        x1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.x);
        y1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.y);
        lb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);
        rb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        rs1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_stick_button);
        ls1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_stick_button);
        a2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        b2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        x2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        y2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        lb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_bumper);
        rb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);
        dpadUp2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_up);
        dpadDown2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_down);
        dpadLeft2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_left);
        dpadRight2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_right);
        onEnteredUpstageEvaluator = new OnActivatedEvaluator(this::isUpstage);
        onEnteredBackstageEvaluator = new OnActivatedEvaluator(this::isBackstage);

        //arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        //extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
//        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
//        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
//        wrist = hardwareMap.get(Servo.class, "redE3");
        //launcher = hardwareMap.get(DcMotorEx.class, "launcherE2");
//        launcherWrist = hardwareMap.get(Servo.class, "launcher");
//        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
//        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);

        //launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launcher.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void autonomousInit(AlliancePose alliancePose, NibusAutonomousPlan autonomousPlan) {
        this.alliancePose = alliancePose;
        this.autonomousPlan = autonomousPlan;
        drive.setPoseEstimate((allianceColor.getAbsoluteFieldPose(alliancePose)));

//        wrist.setPosition(.8);
        sleep(1000);
        autoHomeCollectorLoop();
//        wrist.setPosition(1);
        grabberInit();

        visionPortal.setActiveCamera(backCamera);
        visionPortal.setProcessorEnabled(propFinder, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void teleopInit(NibusSaveState saveState) {
//        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extender.setTargetPosition(0);
//        extender.setTargetPositionTolerance(ARM_TOLERANCE);
//        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start 0,0'd
        drive.setPoseEstimate(new Pose2d());
        autoHomeCollectorLoopFast();

        visionPortal.setActiveCamera(frontCamera);
        visionPortal.setProcessorEnabled(propFinder, false);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void startup(NibusState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
    }

    public boolean evaluate() {
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();

        evaluateFieldPosition();
        evaluatePositioningSystems();
        controlScoringSystems();

        NibusState currentState = state;
        NibusState nextState = state;
        switch (state) {
            case MANUAL_DRIVE:
                nextState = evaluateDrivingAndScoring();
                break;
            case DRIVE_DIRECT_TO_POSE:
                nextState = evaluateDriveDirectToPosition();
                break;
            case AUTONOMOUSLY_DRIVING:
                nextState = evaluateDrivingAutonomously();
                break;
            case DETECT_POSE_FROM_APRIL_TAG:
                nextState = evaluateDetectPoseFromAprilTag();
                break;
            case DETECT_ALLIANCE_MARKER:
                nextState = evaluateDetectAllianceMarker();
                break;
            default:
                break;
        }
        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        boolean indicatorState = state == NibusState.DETECT_POSE_FROM_APRIL_TAG;
        indicator1Green.setState(!indicatorState);
        indicator1Red.setState(indicatorState);

        runTelemetry();

        return state != NibusState.HALT_OPMODE;
    }

    private void evaluateFieldPosition() {
        if (onEnteredUpstageEvaluator.evaluate()) {
            visionPortal.setActiveCamera(backCamera);
            poseQueue.clear();
        }

        if (onEnteredBackstageEvaluator.evaluate()) {
            visionPortal.setActiveCamera(frontCamera);
            poseQueue.clear();
        }
    }

    private void evaluatePositioningSystems() {
        boolean usingBackCamera = visionPortal.getActiveCamera().equals(backCamera);
        double cameraDistanceOffset = usingBackCamera ? BACK_CAMERA_OFFSET_INCHES : FRONT_CAMERA_OFFSET_INCHES;
        double cameraAngleOffset = usingBackCamera ? Math.PI : 0;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                Pose2d estimatedPose = calculateRobotPose(detection, cameraDistanceOffset, cameraAngleOffset);
                if (poseQueue.size() >= QUEUE_CAPACITY) {
                    poseQueue.poll(); // Remove the oldest element
                }
                poseQueue.offer(estimatedPose);
            }
        }

        // Calculate average and variance
        if (poseQueue.size() == QUEUE_CAPACITY) {
            Pose2d averagePose = calculateAveragePose(poseQueue);
            Pose2d variancePose = calculateVariancePose(poseQueue, averagePose);

            // Thresholds for variance, adjust as needed
            double translationVarianceThreshold = 1.0;
            double headingVarianceThreshold = Math.PI / 32;
            if (variancePose.getX() <= translationVarianceThreshold && variancePose.getY() <= translationVarianceThreshold && variancePose.getHeading() <= headingVarianceThreshold) {
                drive.setPoseEstimate(averagePose);
                hasAprilTagFieldPosition = true;
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

    private NibusState evaluateDrivingAndScoring() {
        if (x1PressedEvaluator.evaluate() || (approachTarget != null && y2PressedEvaluator.evaluate())) {
            if (approachTarget == null) {
                if (isUpstage()) {
                    approachTarget = allianceColor.getMainCollectApproach();
                } else if (isBackstage()) {
                    approachTarget = allianceColor.getScoringApproach();
                }
            } else {
                approachTarget = null;
            }
        }

        controlDrivingFromGamepad();
        evaluateScoringManualControls();

        // Safe travels
//        if (y1PressedEvaluator.evaluate()) {
//            launchingAirplane = true;
//            launchingAirplaneTimeMillis = (int) timeSinceStart.milliseconds();
//        }

        if (b1PressedEvaluator.evaluate()) {
            targetTag = null;
            return NibusState.DETECT_POSE_FROM_APRIL_TAG;
        }

        if (x2PressedEvaluator.evaluate()) {
            targetTag = allianceColor.getAprilTagForScoringPosition(CenterStageBackdropPosition.LEFT);
        } else if (y2PressedEvaluator.evaluate()) {
            targetTag = allianceColor.getAprilTagForScoringPosition(CenterStageBackdropPosition.CENTER);
        } else if (b2PressedEvaluator.evaluate()) {
            targetTag = allianceColor.getAprilTagForScoringPosition(CenterStageBackdropPosition.RIGHT);
        } else if (x2PressedEvaluator.evaluate()) {
            targetTag = null;
        }

        if (b1PressedEvaluator.evaluate() && hasAprilTagFieldPosition) {
            poseTarget = nextNibusApproach.Pose;
            nextNibusApproach = nextNibusApproach.nextPose();
            return NibusState.DRIVE_DIRECT_TO_POSE;
        }

        // And good luck
        if (rs1PressedEvaluator.evaluate()) {
            endgameLiftStage = Math.min(3, endgameLiftStage + 1);
            collectorState = endgameLiftStage(endgameLiftStage);
        } else if (ls1PressedEvaluator.evaluate()) {
            endgameLiftStage = Math.max(0, endgameLiftStage - 1);
            collectorState = endgameLiftStage(endgameLiftStage);
        }

        if (dpadUp2PressedEvaluator.evaluate()) {
            armTrimIncrements = Math.min(armTrimIncrements + 1, ARM_MAX_TRIM_INCREMENTS);
        }
        if (dpadDown2PressedEvaluator.evaluate()) {
            armTrimIncrements = Math.max(armTrimIncrements - 1, -ARM_MAX_TRIM_INCREMENTS);
        }
        if (dpadLeft2PressedEvaluator.evaluate()) {
            wristTrimIncrements = Math.min(wristTrimIncrements + 1, WRIST_MAX_TRIM_INCREMENTS);
        }
        if (dpadRight2PressedEvaluator.evaluate()) {
            wristTrimIncrements = Math.max(wristTrimIncrements - 1, -WRIST_MAX_TRIM_INCREMENTS);
        }

        return NibusState.MANUAL_DRIVE;
    }

    private CollectorState endgameLiftStage(int stageNumber) {
        switch (stageNumber) {
            case 0:
                return CollectorState.DRIVING_SAFE;
            case 1:
                return CollectorState.HANG1;
            case 2:
                return CollectorState.HANG2;
            case 3:
                return CollectorState.HANG3;
            default:
                return CollectorState.DRIVING_SAFE;
        }
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
        return new Pose2d(
                Range.clip(error.getX() * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                Range.clip(error.getY() * SPEED_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                Range.clip(error.getHeading() * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
    }

    private NibusState evaluateDriveDirectToPosition() {
        if (y1PressedEvaluator.evaluate() || poseTarget == null) {
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        drive.setWeightedDrivePower(getPoseTargetAutoDriveControl(poseTarget));

        if (settledAtPoseTarget.evaluate()) {
            poseTarget = null;
            return NibusState.MANUAL_DRIVE;
        }

        return NibusState.DRIVE_DIRECT_TO_POSE;
    }

    private NibusState evaluateDrivingAutonomously() {
        if (commandSequence.size() == 0) {
            NibusState _continuationState = continuationState;
            continuationState = null;
            return _continuationState == null ? NibusState.MANUAL_DRIVE : _continuationState;
        }

        if (currentCommand == null) {
            currentCommandTime.reset();
            currentCommand = commandSequence.get(0);

            if (currentCommand.BlueGrabberState != null) {
                blueGrabberState = currentCommand.BlueGrabberState;
            }

            if (currentCommand.GreenGrabberState != null) {
                greenGrabberState = currentCommand.GreenGrabberState;
            }

            if (currentCommand.CollectorState != null) {
                Log.d("evaluateDrivingAutonomously", "Setting collector state");
                collectorState = currentCommand.CollectorState;
            }

            if (currentCommand.TrajectoryCreator != null) {
                Log.d("evaluateDrivingAutonomously", "Following trajectory async");
                drive.followTrajectoryAsync(currentCommand.TrajectoryCreator.create());
            }
        }

        boolean collectorSettled = currentCommand.CollectorState == null || armAndExtenderSettled();
        boolean driveCompleted = currentCommand.TrajectoryCreator == null || !drive.isBusy();
        boolean minTimeElapsed = currentCommandTime.milliseconds() > currentCommand.MinTimeMillis;
        boolean commandComplete = minTimeElapsed && collectorSettled && driveCompleted;
        if (commandComplete) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            commandSequence.remove(0);
            currentCommand = null;
        }

        return NibusState.AUTONOMOUSLY_DRIVING;
    }

    private CenterStageAprilTags targetTag = null;

    private NibusState evaluateDetectPoseFromAprilTag() {

        AprilTagDetection approachDetection = null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (targetTag != null && detection.id == targetTag.Id) {
                approachDetection = detection;
                break;
            }

            // Priority to some tags for positional approach...consider removing to focus on scoring approach use case.
            if (detection.id == 2 ||
                    detection.id == 5 ||
                    detection.id == 7 ||
                    detection.id == 10) {
                approachDetection = detection;
                break;
            }
        }

        if (approachDetection != null) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (approachDetection.ftcPose.range - DESIRED_DISTANCE);
            double headingError = Angle.normDelta(Math.toRadians(approachDetection.ftcPose.bearing));
            double yawError = Angle.normDelta(Math.toRadians(approachDetection.ftcPose.yaw));

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double speed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Speed %5.2f, Strafe %5.2f, Turn %5.2f ", speed, strafe, turn);

            if (Math.abs(rangeError) < DISTANCE_ERROR_THRESHOLD &&
                    Math.abs(headingError) < TURN_ERROR_THRESHOLD &&
                    Math.abs(yawError) < STRAFE_ERROR_THRESHOLD) {
                if (approachSettlingTimer == null) {
                    approachSettlingTimer = new ElapsedTime();
                    approachSettlingTimer.reset();
                }
            } else if (approachSettlingTimer != null) {
                approachSettlingTimer.reset();
            }

            drive.setWeightedDrivePower(new Pose2d(speed, strafe, turn));
        } else {
            if (approachSettlingTimer != null) {
                approachSettlingTimer.reset();
            }
            drive.setWeightedDrivePower(new Pose2d());
        }

        if (b1PressedEvaluator.evaluate()) {
            approachSettlingTimer = null;
            targetTag = null;
            return NibusState.MANUAL_DRIVE;
        }

        if (approachSettlingTimer != null && approachSettlingTimer.milliseconds() > APPROACH_SETTLE_TIME_MS) {
            drive.setPoseEstimate(calculateRobotPoseWhenApproachToAprilTagSettled(approachDetection));
            hasAprilTagFieldPosition = true;
            approachSettlingTimer = null;
            targetTag = null;
            return NibusState.MANUAL_DRIVE;
        }

        return NibusState.DETECT_POSE_FROM_APRIL_TAG;
    }

    private Pose2d calculateRobotPoseWhenApproachToAprilTagSettled(AprilTagDetection detection) {
        CenterStageAprilTags tag = CenterStageAprilTags.getTag(detection.id);
        if (tag == null) return null;

        double tagHeading = tag.Pose.getHeading();
        // Assume we are squared to the april tag cause we settled
        double robotXFieldPosition = tag.Pose.getX() + ((DESIRED_DISTANCE + FRONT_CAMERA_OFFSET_INCHES) * Math.cos(tagHeading));
        double robotYFieldPosition = tag.Pose.getY();

        return new Pose2d(robotXFieldPosition, robotYFieldPosition, Angle.norm(tagHeading + Math.PI));
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection, double cameraRobotOffset, double cameraRobotHeadingOffset) {
        CenterStageAprilTags tag = CenterStageAprilTags.getTag(detection.id);
        if (tag == null) return null;

        double yaw = Math.toRadians(detection.ftcPose.yaw);
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double range = detection.ftcPose.range;
        double tagToCameraHeading = Angle.norm(tag.Pose.getHeading() + bearing - yaw);
        double cameraFieldX = tag.Pose.getX() + (range * Math.cos(tagToCameraHeading));
        double cameraFieldY = tag.Pose.getY() + (range * Math.sin(tagToCameraHeading));
        double cameraFieldHeading = Angle.norm(
                tag.Pose.getHeading() + Math.PI + cameraRobotHeadingOffset - yaw);

        double robotFieldX = cameraFieldX - (cameraRobotOffset * Math.cos(cameraFieldHeading));
        double robotFieldY = cameraFieldY - (cameraRobotOffset * Math.sin(cameraFieldHeading));

        return new Pose2d(robotFieldX, robotFieldY, cameraFieldHeading);
    }

    private NibusState evaluateDetectAllianceMarker() {

        // Lazy init the prop finder and vision portal
        if (propFinder == null) {
            framesProcessed = 0;
            leftMidRightVotes[0] = 0;
            leftMidRightVotes[1] = 0;
            leftMidRightVotes[2] = 0;
            alliancePropPosition = null;
            elapsedPropFrameTime.reset();

        }

        if (elapsedPropFrameTime.milliseconds() > FRAME_DELAY_MILLIS && framesProcessed < (DELAY_FRAMES + PROCESS_FRAMES)) {
            elapsedPropFrameTime.reset();
            Object[] redResults = propFinder.topbox(PROP_CAMERA_WIDTH_PIXELS, PROP_CAMERA_HEIGHT_PIXELS, PROP_CAMERA_ROW_COUNT, PROP_CAMERA_COLUMN_COUNT, allianceColor);
            if (redResults.length > 0) {
                if (framesProcessed >= DELAY_FRAMES) {
                    int voteIndex = (int) redResults[1];
                    leftMidRightVotes[voteIndex]++;
                    Log.d("nibusVision", String.format("Vision vote for %d", voteIndex));
                } else {
                    Log.d("nibusVision", "Delaying this frame.");
                }
                framesProcessed++;
            } else {
                Log.d("nibusVision", "No frame processed");
            }
        }

        if (framesProcessed >= DELAY_FRAMES + PROCESS_FRAMES) {
            int leftVotes = leftMidRightVotes[0];
            int midVotes = leftMidRightVotes[1];
            int rightVotes = leftMidRightVotes[2];
            if (leftVotes >= midVotes && leftVotes >= rightVotes) {
                alliancePropPosition = AlliancePropPosition.LEFT;
            } else if (midVotes > leftVotes && midVotes >= rightVotes) {
                alliancePropPosition = AlliancePropPosition.MID;
            } else {
                alliancePropPosition = AlliancePropPosition.RIGHT;
            }

            Log.d("evaluateDetectAllianceMarker", String.format("Detected %s", alliancePropPosition.name()));

            if (alliancePose != null && autonomousPlan != null) {
                planAutonomousAfterPropDetect(autonomousPlan.getParkLocation(allianceColor));
                return NibusState.AUTONOMOUSLY_DRIVING;
            } else {
                return NibusState.MANUAL_DRIVE;
            }
        }

        return NibusState.DETECT_ALLIANCE_MARKER;
    }

    private void planAutonomousAfterPropDetect(Vector2d endParkingLocation) {
        Vector2d targetLocation = alliancePose.getPixelTargetPosition(allianceColor, alliancePropPosition);
        double targetHeading = 0.0;
        switch (allianceColor) {
            case RED:
                targetHeading = Math.toRadians(270 - 135);
                break;
            case BLUE:
                targetHeading = Math.toRadians(90 + 135);
                break;
            default:
                targetHeading = 0;
        }

        Pose2d targetPose = new Pose2d(targetLocation.getX(), targetLocation.getY(), targetHeading);
        Pose2d approachPose = NibusHelpers.collectApproachPose(targetPose);

        Vector2d waypoint1 = allianceColor.getMiddleLaneAudienceWaypoint();
        Vector2d waypoint2 = allianceColor.getScoringPreApproachLocation();
        Vector2d waypoint3 = allianceColor.getScoringApproachLocation();

        setAutonomousCommands(NibusState.MANUAL_DRIVE,
                new NibusAutonomousCommand(CollectorState.CLOSE_COLLECTION),
                new NibusAutonomousCommand(
                        () -> buildAutonomousTrajectoryFromHere(approachPose)),
                new NibusAutonomousCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED),
                new NibusAutonomousCommand(CollectorState.DRIVING_SAFE),
                new NibusAutonomousCommand(
                        () -> buildAutonomousTrajectoryFromHere(waypoint1, 0)),
                new NibusAutonomousCommand(
                        () -> buildAutonomousTrajectoryFromHere(waypoint2, 0)),
                new NibusAutonomousCommand(
                        () -> buildAutonomousTrajectoryFromHere(waypoint3, 0)),
                new NibusAutonomousCommand(CollectorState.HIGH_SCORING),
                new NibusAutonomousCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.NOT_GRABBED),
                new NibusAutonomousCommand(CollectorState.DRIVING_SAFE),
                new NibusAutonomousCommand(
                        () -> buildAutonomousTrajectoryFromHere(endParkingLocation, 0)));
    }

    private Trajectory buildAutonomousTrajectoryFromHere(Vector2d vector2d, double heading) {
        return buildAutonomousTrajectoryFromHere(convertToPose(vector2d, heading));
    }

    private Trajectory buildAutonomousTrajectoryFromHere(Pose2d pose2d) {
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(pose2d,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .66, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    private void setAutonomousCommands(NibusState _continuationState, NibusAutonomousCommand ... commands) {
        commandSequence.clear();
        commandSequence.addAll(Arrays.asList(commands));
        continuationState = _continuationState;
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad) {
        Vector2d inputFieldDirection = NibusHelpers.headlessLeftStickFieldDirection(gamepad1, allianceColor.OperatorHeadingOffset, latestPoseEstimate.getHeading());
        double scale = gamepad.right_trigger > 0.2 ? 0.3 : 0.8; // Slow mode scaling
        double scaledRobotX = inputFieldDirection.getX() * scale;
        double scaledRobotY = inputFieldDirection.getY() * scale;
        double scaledRotation = -gamepad.right_stick_x * scale;
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private void controlDrivingFromGamepad() {

        Pose2d controlPose;

        // Navigation mode
        if (approachTarget == null) {

            controlPose = getScaledHeadlessDriverInput(gamepad1);

            // Lane lock
            if (hasPositionEstimate() && gamepad1.x) {
                int closestLaneY = CenterStageConstants.getClosestLane(latestPoseEstimate.getY());
                double errorY = closestLaneY - latestPoseEstimate.getY();
                double errorHeading = Angle.normDelta(0 - latestPoseEstimate.getHeading());

                double laneLockY = Range.clip(errorY * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double laneLockRotation = Range.clip(errorHeading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                controlPose = controlPose.plus(new Pose2d(0, laneLockY, laneLockRotation));
            }

            // TODO: crossing pull

        // Finesse mode
        } else {
            double approachAngleOffset = (-gamepad1.left_stick_y) * approachTarget.ApproachHeadingRange;
            Pose2d approachTargetPoseWithDriverOffset = NibusHelpers.collectApproachPose(
                    approachTarget.Pose.minus(new Pose2d(0, 0, approachAngleOffset)));
            controlPose = getPoseTargetAutoDriveControl(approachTargetPoseWithDriverOffset);
            controlPose = controlPose.plus(getScaledHeadlessDriverInput(gamepad2).div(2));
        }

        drive.setWeightedDrivePower(controlPose);
    }

    private void controlScoringSystems() {
//        greenGrabber.setPosition(greenGrabberState.ServoPosition);
//        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        double wristPositionToApply = Math.min(1, Math.max(0, collectorState.WristPosition + (wristTrimIncrements * WRIST_SERVO_TRIM_INCREMENT)));
//        wrist.setPosition(wristPositionToApply);
        extendLengthCm = collectorState.ExtenderPosition;
        armTargetDegrees = collectorState.ArmPosition;

        double trimmedArmTargetDegrees = armTargetDegrees + (armTrimIncrements * ARM_DEGREE_TRIM_INCREMENT);
        target = computeArmTickTarget(trimmedArmTargetDegrees);
        armcontrol.setPID(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);
        armcontrol.setTolerance(ARM_TOLERANCE);

//        currentArmPosition = arm_motor0.getCurrentPosition();
        double armpower = armcontrol.calculate(currentArmPosition, target);

        //Set extension
        extendTicTarget = (int) computeExtenderTickTarget(extendLengthCm);
//        currentExtenderPosition = extender.getCurrentPosition();
        //extender.setTargetPosition(extendTicTarget);

        if(Math.abs(armpower) < 0.01) armpower = 0;
//        if(armpower < 0 && armMin.isPressed()) armpower = 0;
        //arm_motor0.setPower(armpower);
        //extender.setPower(EXTENDER_POWER);

        if (launchingAirplane) {
//            launcherWrist.setPosition(LAUNCH_WRIST_POSITION);
            int launchSequenceTimeMillis = (int) timeSinceStart.milliseconds() - launchingAirplaneTimeMillis;
            if (launchSequenceTimeMillis > 1000) {
                //launcher.setPower(1);
            }

            if (launchSequenceTimeMillis > 2000) {
                //launcher.setPower(0);
                launchingAirplane = false;
            }
        }

        telemetry.addData("Armpower", armpower);
    }

    private double computeArmTickTarget(double degreesOffsetFromHorizontal) {
        return (((Math.min(degreesOffsetFromHorizontal, ARM_MAX_ANGLE)) - ARM_DEGREE_OFFSET_FROM_HORIZONTAL) * ARM_TICKS_PER_DEGREE);
    }

    private double computeExtenderTickTarget(double extendLength) {
        return ((Math.min(extendLength, EXTENDER_MAX_LENGTH)) * EXTENDER_TICS_PER_CM);
    }

    private boolean armAndExtenderSettled() {
        return true; // TODO: reenable
//        int armErrorTicks = Math.abs((int) target - arm_motor0.getCurrentPosition());
//        int extenderErrorTicks = Math.abs((int) extendTicTarget - extender.getCurrentPosition());
//        return armErrorTicks < 20 && extenderErrorTicks < 20;
    }

    private void evaluateScoringManualControls() {
        if (a1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
            greenGrabberState = greenGrabberState.toggle();
        }

        if (lb1PressedEvaluator.evaluate()) {
            greenGrabberState = greenGrabberState.toggle();
        }

        if (rb1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
        }

//        if (a2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.CLOSE_COLLECTION;
//        } else if (b2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.FAR_COLLECTION;
//        } else if (x2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.DRIVING_SAFE;
//        } else if (y2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.LOW_SCORING;
//        } else if (rb2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.HIGH_SCORING;
//        } else if (lb2PressedEvaluator.evaluate()) {
//            collectorState = CollectorState.DRIVING_SAFE;
//        }
        telemetry.addData("Collector state", collectorState);
//        telemetry.addData("Wrist pos:",wrist.getPosition());
    }

    private void grabberInit() {
        // Initial short cycle to visually indicate ready to load pixels
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
//        greenGrabber.setPosition(greenGrabberState.ServoPosition);
//        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
//        greenGrabber.setPosition(greenGrabberState.ServoPosition);
//        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        // Then hold open for 3 seconds before closing
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
//        greenGrabber.setPosition(greenGrabberState.ServoPosition);
//        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(3000);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
//        greenGrabber.setPosition(greenGrabberState.ServoPosition);
//        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void runTelemetry() {
        telemetry.addData("x", latestPoseEstimate.getX());
        telemetry.addData("y", latestPoseEstimate.getY());
        telemetry.addData("heading", latestPoseEstimate.getHeading());
        telemetry.update();
    }

    private void autoHomeCollectorLoop() {
        return; //TODO reenable
//        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extender.setDirection(DcMotorEx.Direction.FORWARD);
//        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        while (!extenderMin.isPressed()) {
//            extender.setPower(-0.4);
//        }
//        while (extenderMin.isPressed()) {
//            extender.setPower(0.4);
//        }
//        sleep(500);

//        while (!extenderMin.isPressed()) {
//            extender.setPower(-0.1);
//        }
//        extender.setPower(0);
//        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extender.setTargetPosition(0);
//        extender.setTargetPositionTolerance(ARM_TOLERANCE);
//        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //home arm
//        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        while (!armMin.isPressed()){
//            arm_motor0.setPower(-0.65);
//        }
//        while (armMin.isPressed()){
//            arm_motor0.setPower(0.65);
//        }
//        sleep(500);

//        while (!armMin.isPressed()) {
//            arm_motor0.setPower(-0.05);
//        }
//        arm_motor0.setPower(0);
//        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void autoHomeCollectorLoopFast() {
        return; //TODO reenable
//        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extender.setDirection(DcMotorEx.Direction.FORWARD);
//        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        while (!extenderMin.isPressed()) {
//            extender.setPower(-0.4);
//        }

//        extender.setPower(0);
//        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extender.setTargetPosition(0);
//        extender.setTargetPositionTolerance(ARM_TOLERANCE);
//        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //home arm
//        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        while (!armMin.isPressed()){
//            arm_motor0.setPower(-0.30);
//        }

//        arm_motor0.setPower(0);
//        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public int getArmPosition() {
        return currentArmPosition;
    }

    public int getExtenderPosition() {
        return currentExtenderPosition;
    }

    public CollectorState getCollectorState() {
        return collectorState;
    }

    public BlueGrabberState getBlueGrabberState() {
        return blueGrabberState;
    }

    public GreenGrabberState getGreenGrabberState() {
        return greenGrabberState;
    }

    public Pose2d convertToPose(Vector2d vector2d, double heading) {
        return new Pose2d(vector2d.getX(), vector2d.getY(), heading);
    }

    private boolean isEndgame() {
        return timeSinceStart.milliseconds() > END_GAME_BEGINS_MILLIS;
    }

    private boolean hasPositionEstimate() {
        return hasAprilTagFieldPosition && latestPoseEstimate != null;
    }

    private boolean isUpstage() {
        return hasPositionEstimate() && latestPoseEstimate.getX() < CenterStageConstants.UPSTAGE_FIELD_X;
    }

    private boolean isBackstage() {
        return hasPositionEstimate() && latestPoseEstimate.getX() > CenterStageConstants.BACKSTAGE_FIELD_X;
    }

    private boolean isInterstage() {
        return hasPositionEstimate() && !isUpstage() && !isBackstage();
    }
}
