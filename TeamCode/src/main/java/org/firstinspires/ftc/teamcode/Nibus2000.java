package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NibusConstants.*;

import android.graphics.PostProcessor;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Nibus2000 {

    private final int armStartingTickOffset = 0; //TODO: Reimplement saved state
    private final int extenderStartingTickOffset = 0;
    private final SampleMecanumDrive drive;
    private final TouchSensor armMin;
    private final TouchSensor extenderMin;
    private final PIDController armcontrol;

    public double extendLengthCm = 0;
    public int extendTicTarget = 0;
    public double armTargetDegrees = 0;
    public double target = 0.0;
    private final DcMotorEx arm_motor0;
    private final DcMotorEx extender;
    private final DcMotorEx launcher;
    Servo greenGrabber;
    Servo blueGrabber;
    Servo wrist;
    Servo launcherWrist;
    private CollectorState collectorState = CollectorState.DRIVING_SAFE;
    private final OnActivatedEvaluator a1PressedEvaluator;
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
    private final OnActivatedEvaluator rs1PressedEvaluator;
    private final OnActivatedEvaluator ls1PressedEvaluator;
    private BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
    private GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private AllianceColor allianceColor;
    private AlliancePose alliancePose;
    private NibusState state;
    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private VisionPortal frontVisionPortal;
    private VisionPortal backVisionPortal;
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
    private PoseOfInterest nextPoseOfInterest = PoseOfInterest.RANDOM_POINT;

    public Nibus2000(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.state = NibusState.MANUAL_DRIVE;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        indicator1Red = hardwareMap.get(DigitalChannel.class, "indicator1red");
        indicator1Green = hardwareMap.get(DigitalChannel.class, "indicator1green");
        indicator1Red.setMode(DigitalChannel.Mode.OUTPUT);
        indicator1Green.setMode(DigitalChannel.Mode.OUTPUT);

        a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
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

        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
        wrist = hardwareMap.get(Servo.class, "redE3");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherE2");
        launcherWrist = hardwareMap.get(Servo.class, "launcher");
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void autonomousInit(AlliancePose alliancePose, NibusAutonomousPlan autonomousPlan) {
        this.alliancePose = alliancePose;
        this.autonomousPlan = autonomousPlan;
        drive.setPoseEstimate((allianceColor.getAbsoluteFieldPose(alliancePose)));

        wrist.setPosition(.8);
        sleep(1000);
        autoHomeCollectorLoop();
        wrist.setPosition(1);
        grabberInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void teleopInit(NibusSaveState saveState) {
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(ARM_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start 0,0'd
        drive.setPoseEstimate(new Pose2d());

// TODO: Reimplement save state
//        if (saveState != null) {
//            drive.setPoseEstimate(saveState.Pose);
//            armStartingTickOffset = saveState.ArmPosition;
//            extenderStartingTickOffset = saveState.ExtenderPosition;
//            Log.d("Nibus2000", String.format("Setting arm offset ticks: %d", armStartingTickOffset));
//            Log.d("Nibus2000", String.format("Setting extender offset ticks: %d", extenderStartingTickOffset));
//            blueGrabberState = saveState.BlueGrabberState;
//            greenGrabberState = saveState.GreenGrabberState;
//            collectorState = saveState.CollectorState;
//        } else {
            wrist.setPosition(1);
            autoHomeCollectorLoopFast();
//        }
    }

    public void startup(NibusState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
    }

    public boolean evaluate() {
        // Update drive on every cycle to keep the odometry position in sync at all times.
        drive.update();
        controlScoringSystems();

        NibusState currentState = state;
        NibusState nextState = state;
        latestPoseEstimate = drive.getPoseEstimate();
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

    private boolean indicator1State = false;
    private boolean blinkinIndicatorState = false;

    private NibusState evaluateDrivingAndScoring() {
        controlDrivingFromGamepad();
        evaluateScoringManualControls();

        // Safe travels
//        if (y1PressedEvaluator.evaluate()) {
//            launchingAirplane = true;
//            launchingAirplaneTimeMillis = (int) timeSinceStart.milliseconds();
//        }

        if (y1PressedEvaluator.evaluate()) {
            return NibusState.DETECT_POSE_FROM_APRIL_TAG;
        }

        if (b1PressedEvaluator.evaluate() && hasAprilTagFieldPosition) {
            driveToPoseTarget = nextPoseOfInterest.Pose;
            nextPoseOfInterest = nextPoseOfInterest.nextPose();
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

    private Pose2d driveToPoseTarget = null;
    private static final double DRIVE_TO_POSE_THRESHOLD = 1.0f;
    private NibusState evaluateDriveDirectToPosition() {
        if (y1PressedEvaluator.evaluate() || driveToPoseTarget == null) {
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        double errorX = driveToPoseTarget.getX() - latestPoseEstimate.getX();
        double errorY = driveToPoseTarget.getY() - latestPoseEstimate.getY();
        double errorHeading = Angle.normDelta(driveToPoseTarget.getHeading() - latestPoseEstimate.getHeading());

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double x = Range.clip(errorX * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double y = Range.clip(errorY * SPEED_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        double theta = Range.clip(errorHeading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        drive.setWeightedDrivePower(new Pose2d(x, y, theta));

        double errorMagnitude = Math.hypot(errorX, errorY);
        if (errorMagnitude < DRIVE_TO_POSE_THRESHOLD) {
            if (approachSettlingTimer == null) {
                approachSettlingTimer = new ElapsedTime();
                approachSettlingTimer.reset();
            }
        } else if (approachSettlingTimer != null) {
            approachSettlingTimer.reset();
        }

        if (approachSettlingTimer != null && approachSettlingTimer.milliseconds() > APPROACH_SETTLE_TIME_MS) {
            approachSettlingTimer = null;
            driveToPoseTarget = null;
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

    final double DISTANCE_ERROR_THRESHOLD = 0.5;
    final double STRAFE_ERROR_THRESHOLD = Math.PI / 32;
    final double TURN_ERROR_THRESHOLD = Math.PI / 32;
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.1  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  4/Math.PI ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  4/Math.PI  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = Math.PI / 2;   //  Clip the turn speed to this max value (adjust for your robot)
    final int APPROACH_SETTLE_TIME_MS = 500;

    private ElapsedTime approachSettlingTimer = null;
    private boolean hasAprilTagFieldPosition = false;
    private NibusState evaluateDetectPoseFromAprilTag() {

        if (aprilTagProcessor == null) {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
        }

        if (frontVisionPortal == null) {
            frontVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
        }

        frontVisionPortal.resumeStreaming();

        AprilTagDetection approachDetection = null;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 2) {
                approachDetection = detection;
                // Log tag detection data
                telemetry.addData("Detected target",
                        "X: %.1f, Y: %.1f, Z: %.1f\nYaw: %.1f, Pitch: %.1f\nRange: %.1f",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z,
                        detection.ftcPose.yaw,
                        detection.ftcPose.pitch,
                        detection.ftcPose.range);

                // Calculate robot's center position
                Pose2d robotCenterPose = calculateCameraPose(detection.ftcPose);
                telemetry.addData("Camera Center", "X: %.1f, Y: %.1f",
                        robotCenterPose.getX(), robotCenterPose.getY());
                drive.setPoseEstimate(robotCenterPose);
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

        if (y1PressedEvaluator.evaluate()) {
            frontVisionPortal.stopStreaming();
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        if (approachSettlingTimer != null && approachSettlingTimer.milliseconds() > APPROACH_SETTLE_TIME_MS) {
            drive.setPoseEstimate(new Pose2d(64.0 - 12 - 7, 35.0, 0));
            hasAprilTagFieldPosition = true;
            frontVisionPortal.stopStreaming();
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        return NibusState.DETECT_POSE_FROM_APRIL_TAG;
    }

    private Pose2d calculateCameraPose(AprilTagPoseFtc ftcPose) {
        final double TAG_X = 64.0; // Tag's X position on field
        final double TAG_Y = 35.0; // Tag's Y position on field
        final double TAG_HEADING = Math.toRadians(180);

        // Calculate new heading
        double tagToCameraHeading = Angle.norm(TAG_HEADING - Angle.normDelta(Math.toRadians(ftcPose.yaw)));
        double tagToCameraDistance = ftcPose.range;

        telemetry.addData("tagToRobotHeading", Math.toDegrees(tagToCameraHeading));
        telemetry.addData("tagToRobotDistance", tagToCameraDistance);

        // Compute the x, y location
        double cameraOffsetFieldX = tagToCameraDistance * Math.cos(tagToCameraHeading);
        double cameraOffsetFieldY = tagToCameraDistance * Math.sin(tagToCameraHeading);

        telemetry.addData("cameraOffsetFieldX", cameraOffsetFieldX);
        telemetry.addData("cameraOffsetFieldY", cameraOffsetFieldY);

        double cameraFieldX = TAG_X + cameraOffsetFieldX;
        double cameraFieldY = TAG_Y + cameraOffsetFieldY;
        double cameraFieldHeading = Angle.norm(tagToCameraHeading + Math.PI);

        telemetry.addData("cameraFieldX", cameraFieldX);
        telemetry.addData("cameraFieldY", cameraFieldY);
        telemetry.addData("cameraFieldHeading", cameraFieldHeading);

        return new Pose2d(cameraFieldX, cameraFieldY, cameraFieldHeading);
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

            propFinder = new WindowBoxesVisionProcessor();
        }

        if (backVisionPortal == null) {
            backVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 2"), propFinder);
        }

        // Idempotent
        backVisionPortal.resumeStreaming();

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
            backVisionPortal.stopStreaming();
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
        Pose2d approachPose = calculateApproachPose(targetPose, -8);

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

    private void controlDrivingFromGamepad() {
        // Get the current heading of the robot
        double robotHeading = drive.getPoseEstimate().getHeading();

        // Get the gamepad stick inputs and normalize them
        double rawGamepadY = gamepad1.left_stick_y;
        double rawGamepadX = gamepad1.left_stick_x;
        double normalizedGamepadY = -rawGamepadY; // Inverting Y to normalize direction
        double normalizedGamepadX = rawGamepadX;

        telemetry.addData("robotHeading", robotHeading);
        telemetry.addData("rawGamepadY", rawGamepadY);
        telemetry.addData("rawGamepadX", rawGamepadX);
        telemetry.addData("normalizedGamepadY", normalizedGamepadY);
        telemetry.addData("normalizedGamepadX", normalizedGamepadX);

        // Calculate the magnitude and direction of the gamepad input
        double inputMagnitude = Math.hypot(normalizedGamepadX, normalizedGamepadY);
        double operatorRelativeStickHeading = Angle.norm(Math.atan2(normalizedGamepadY, normalizedGamepadX) - (Math.PI / 2));
        double operatorFieldStickHeading = Angle.norm(operatorRelativeStickHeading + allianceColor.OperatorHeadingOffset);
        double robotRelativeTranslationHeading = Angle.norm(operatorFieldStickHeading - robotHeading);

        telemetry.addData("inputMagnitude", inputMagnitude);
        telemetry.addData("operatorRelativeStickHeading", operatorRelativeStickHeading);
        telemetry.addData("operatorFieldStickHeading", operatorFieldStickHeading);
        telemetry.addData("robotRelativeTranslationHeading", robotRelativeTranslationHeading);

        // Convert from gamepad stick XY polar to the field XY system.
        double robotX = inputMagnitude * Math.cos(robotRelativeTranslationHeading);
        double robotY = inputMagnitude * Math.sin(robotRelativeTranslationHeading);

        telemetry.addData("robotX", robotX);
        telemetry.addData("robotY", robotY);

        // Apply scaled inputs for driving
        double scale = gamepad1.right_trigger > 0.2 ? 0.3 : 1.0; // Slow mode scaling
        double scaledRobotX = robotX * scale;
        double scaledRobotY = robotY * scale;
        double scaledRotation = -gamepad1.right_stick_x * scale;

        telemetry.addData("scaledRobotX", scaledRobotX);
        telemetry.addData("scaledFieldY", scaledRobotY);
        telemetry.addData("scaledRotation", scaledRotation);

        Pose2d drivePower = new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);

        // Uncomment the following line to actually run the motors when ready
        drive.setWeightedDrivePower(drivePower);
    }

    private void controlScoringSystems() {
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        double wristPositionToApply = Math.min(1, Math.max(0, collectorState.WristPosition + (wristTrimIncrements * WRIST_SERVO_TRIM_INCREMENT)));
        wrist.setPosition(wristPositionToApply);
        extendLengthCm = collectorState.ExtenderPosition;
        armTargetDegrees = collectorState.ArmPosition;

        double trimmedArmTargetDegrees = armTargetDegrees + (armTrimIncrements * ARM_DEGREE_TRIM_INCREMENT);
        target = computeArmTickTarget(trimmedArmTargetDegrees);
        armcontrol.setPID(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);
        armcontrol.setTolerance(ARM_TOLERANCE);

        currentArmPosition = arm_motor0.getCurrentPosition();
        double armpower = armcontrol.calculate(currentArmPosition, target);

        //Set extension
        extendTicTarget = (int) computeExtenderTickTarget(extendLengthCm);
        currentExtenderPosition = extender.getCurrentPosition();
        extender.setTargetPosition(extendTicTarget);

        if(Math.abs(armpower) < 0.01) armpower = 0;
        if(armpower < 0 && armMin.isPressed()) armpower = 0;
        arm_motor0.setPower(armpower);
        extender.setPower(EXTENDER_POWER);

        if (launchingAirplane) {
            launcherWrist.setPosition(LAUNCH_WRIST_POSITION);
            int launchSequenceTimeMillis = (int) timeSinceStart.milliseconds() - launchingAirplaneTimeMillis;
            if (launchSequenceTimeMillis > 1000) {
                launcher.setPower(1);
            }

            if (launchSequenceTimeMillis > 2000) {
                launcher.setPower(0);
                launchingAirplane = false;
            }
        }

        telemetry.addData("Armpower", armpower);
    }

    private double computeArmTickTarget(double degreesOffsetFromHorizontal) {
        return (((Math.min(degreesOffsetFromHorizontal, ARM_MAX_ANGLE)) - ARM_DEGREE_OFFSET_FROM_HORIZONTAL) * ARM_TICKS_PER_DEGREE) - armStartingTickOffset;
    }

    private double computeExtenderTickTarget(double extendLength) {
        return ((Math.min(extendLength, EXTENDER_MAX_LENGTH)) * EXTENDER_TICS_PER_CM) - extenderStartingTickOffset;
    }

    private boolean armAndExtenderSettled() {
        int armErrorTicks = Math.abs((int) target - arm_motor0.getCurrentPosition());
        int extenderErrorTicks = Math.abs((int) extendTicTarget - extender.getCurrentPosition());
        return armErrorTicks < 20 && extenderErrorTicks < 20;
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

        if (a2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.CLOSE_COLLECTION;
        } else if (b2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.FAR_COLLECTION;
        } else if (x2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.DRIVING_SAFE;
        } else if (y2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.LOW_SCORING;
        } else if (rb2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.HIGH_SCORING;
        } else if (lb2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.DRIVING_SAFE;
        }
        telemetry.addData("Collector state", collectorState);
        telemetry.addData("Wrist pos:",wrist.getPosition());
    }

    private void grabberInit() {
        // Initial short cycle to visually indicate ready to load pixels
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        // Then hold open for 3 seconds before closing
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(3000);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void runTelemetry() {
        telemetry.addData("x", latestPoseEstimate.getX());
        telemetry.addData("y", latestPoseEstimate.getY());
        telemetry.addData("heading", latestPoseEstimate.getHeading());
        telemetry.update();
    }

    private void autoHomeCollectorLoop() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.4);
        }
        while (extenderMin.isPressed()) {
            extender.setPower(0.4);
        }
        sleep(500);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.1);
        }
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(ARM_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //home arm
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.65);
        }
        while (armMin.isPressed()){
            arm_motor0.setPower(0.65);
        }
        sleep(500);

        while (!armMin.isPressed()) {
            arm_motor0.setPower(-0.05);
        }
        arm_motor0.setPower(0);
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void autoHomeCollectorLoopFast() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.4);
        }

        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(ARM_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //home arm
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.30);
        }

        arm_motor0.setPower(0);
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public Pose2d calculateApproachPose(Pose2d targetPose, double offsetDistance) {
        // Calculate offset components based on target heading
        double offsetX = offsetDistance * Math.cos(targetPose.getHeading());
        double offsetY = offsetDistance * Math.sin(targetPose.getHeading());

        // Calculate the robot's position by subtracting the offset from the target position
        double robotX = targetPose.getX() - offsetX;
        double robotY = targetPose.getY() - offsetY;

        // Return the robot's required pose
        return new Pose2d(robotX, robotY, targetPose.getHeading());
    }

    public Pose2d convertToPose(Vector2d vector2d, double heading) {
        return new Pose2d(vector2d.getX(), vector2d.getY(), heading);
    }

    private boolean isEndgame() {
        return timeSinceStart.milliseconds() > END_GAME_BEGINS_MILLIS;
    }
}
