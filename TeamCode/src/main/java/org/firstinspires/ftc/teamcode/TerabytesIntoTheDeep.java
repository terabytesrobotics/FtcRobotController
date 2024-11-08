package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NibusConstants.ARM_CONTROL_D;
import static org.firstinspires.ftc.teamcode.NibusConstants.ARM_CONTROL_I;
import static org.firstinspires.ftc.teamcode.NibusConstants.ARM_CONTROL_P;
import static org.firstinspires.ftc.teamcode.NibusConstants.ARM_TOLERANCE;
import static org.firstinspires.ftc.teamcode.NibusConstants.EXTENDER_MAX_LENGTH_INCHES;
import static org.firstinspires.ftc.teamcode.NibusConstants.EXTENDER_POWER;
import static org.firstinspires.ftc.teamcode.NibusConstants.EXTENDER_TICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.NibusConstants.sleep;
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private final double PINCER_OPEN = 0.5;
    private final double PINCER_CLOSED = 0.65;
    private final double WRIST_CENTER = 0.582;
    private final double WRIST_RANGE = 0.25;

    // TODO: Trim these GPT fields down to only necessary
    private double fieldTheta;
    private double fieldRadius;
    private double squarenessParameter = 0.75;
    private ElapsedTime lastRadialModeLoopTime = new ElapsedTime();
    private static final double FIELD_SIDE_LENGTH = 60.0; // Not really real field side...it's the drivable square minus margins of robot
    private static final double MIN_FIELD_RADIUS = 40.0;
    private static final double MAX_FIELD_THETA_RATE = Math.toRadians(45);
    private static final double MAX_FIELD_RADIUS_RATE = 12.0;
    // TODO: Trim these GPT fields down to only necessary

    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getIntoTheDeepTagLibrary();
    private final SampleMecanumDrive drive;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final WebcamName frontCamera;
    private final WebcamName backCamera;
    private final DigitalChannel indicator1Red;
    private final DigitalChannel indicator1Green;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private final RevBlinkinLedDriver blinkinLedDriver;
    private TerabytesOpModeState state;
    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private boolean isNewState;
    private Pose2d latestPoseEstimate = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private TerabytesCommand currentCommand = null;
    private final ArrayList<TerabytesCommand> commandSequence = new ArrayList<>();
    private TerabytesOpModeState continuationState = null;
    private Pose2d lastAprilTagFieldPosition = null;
    private double lastAprilTagFieldPositionMillis = 0;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();
    private final boolean debugMode;
    private final AllianceColor allianceColor;

    private final PIDController armControl = new PIDController(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);
    private final DcMotorEx armLeft;
    private final DcMotorEx armRight;
    private final DcMotorEx extender;
    private final TouchSensor armMin;
    private final TouchSensor extenderMin;
    private final Servo wrist;
    private final Servo pincer;
    private final OnActivatedEvaluator a2Evaluator;

    private boolean pincerClosed = true;
    private double extenderTickTarget;

    public TerabytesIntoTheDeep(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.state = TerabytesOpModeState.MANUAL_CONTROL;
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

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        indicator1Red = hardwareMap.get(DigitalChannel.class, "indicator1red");
        indicator1Green = hardwareMap.get(DigitalChannel.class, "indicator1green");
        indicator1Red.setMode(DigitalChannel.Mode.OUTPUT);
        indicator1Green.setMode(DigitalChannel.Mode.OUTPUT);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

        armControl.setTolerance(ARM_TOLERANCE);
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armLeft = hardwareMap.get(DcMotorEx.class, "armE0");
        armRight = hardwareMap.get(DcMotorEx.class, "armE3");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        pincer = hardwareMap.get(Servo.class, "pincer");
        wrist = hardwareMap.get(Servo.class, "wrist");

        a2Evaluator = new OnActivatedEvaluator(() -> gamepad2.a);
    }

    public Pose2d getLatestPoseEstimate() {
        return latestPoseEstimate;
    }

    private void runTelemetry() {
        telemetry.addData("currentState", state.toString());
        if (lastAprilTagFieldPosition != null) {
            telemetry.addData("estimate-x", lastAprilTagFieldPosition.getX());
            telemetry.addData("estimate-y", lastAprilTagFieldPosition.getY());
            telemetry.addData("etimate-heading", lastAprilTagFieldPosition.getHeading());
        }

        if (state == TerabytesOpModeState.RADIAL_DRIVING_MODE) {
            telemetry.addData("Field Theta (deg)", Math.toDegrees(fieldTheta));
            telemetry.addData("Field Radius", fieldRadius);
        }

        telemetry.addData("x", latestPoseEstimate.getX());
        telemetry.addData("y", latestPoseEstimate.getY());
        telemetry.addData("heading", latestPoseEstimate.getHeading());

        telemetry.addData("wrist", (gamepad2.left_stick_x + 1.0) / 2);
        telemetry.addData("pincer", (gamepad2.right_stick_x + 1.0) / 2);
        telemetry.addData("extenderPos", extender.getCurrentPosition());
        telemetry.addData("extenderTarget", extender.getTargetPosition());
        telemetry.addData("extenderMin", extenderMin.isPressed());
        telemetry.addData("armMin", armMin.isPressed());
        telemetry.update();
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

    public void startup(TerabytesOpModeState startupState) {
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
        evaluatePositioningSystems();

        if (state != TerabytesOpModeState.INITIALIZE_TELEOP) {
            extender.setTargetPosition((int) extenderTickTarget);
        }

        // Determine whether there's been a state change
        // Run the state specific control logic
        TerabytesOpModeState currentState = state;
        TerabytesOpModeState nextState = state;
        switch (state) {
            case INITIALIZE_TELEOP:
                nextState = evaluateInitializeTeleop();
                break;
            case MANUAL_CONTROL:
                nextState = evaluateManualControl();
                break;
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence();
                break;
            case HEADLESS_DRIVE:
                nextState = evaluateHeadlessDrivingMode();
                break;
            case RADIAL_DRIVING_MODE:
              nextState = evaluateRadialDrivingMode();
              break;
            case STOPPED_UNTIL_END:
                // Hold the drive still.
                drive.setWeightedDrivePower(new Pose2d());
            default:
                break;
        }

        // Reset the timer so that the state logic can use it to tell how long it's been in that state
        isNewState = currentState != nextState;
        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        runTelemetry();

        return state != TerabytesOpModeState.HALT_OPMODE;
    }

    private void zeroExtender() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(EXTENDER_TICK_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setPower(EXTENDER_POWER);
    }

    private void zeroArmMotors() {
        armLeft.setPower(0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armRight.setPower(0);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void controlArmMotor(DcMotorEx armMotor, int targetPosition) {
        int armPosition = armMotor.getCurrentPosition();
        double armPower = armControl.calculate(armPosition, targetPosition);
        if(Math.abs(armPower) < 0.02) armPower = 0;
        if(armPower < 0 && armMin.isPressed()) armPower = 0;
        telemetry.addData("armMotorPower", "%s: %5.2f", armMotor.getDeviceName(), armPower);
        armMotor.setPower(armPower);
    }

    private TerabytesOpModeState evaluateInitializeTeleop() {
        if (isNewState) {
            zeroExtender();
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setDirection(DcMotorEx.Direction.FORWARD);
            extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        boolean zeroExtender = extenderMin.isPressed();
        if (zeroExtender) {
            extender.setPower(0.0);
            zeroExtender();
        } else {
            extender.setPower(-0.4);
        }

        boolean zeroArm = armMin.isPressed();
        if (zeroArm) {
            armLeft.setPower(0);
            armRight.setPower(0);
            zeroArmMotors();
        } else if (zeroExtender) {
            armLeft.setPower(-0.30);
            armRight.setPower(-0.30);
        }

        if (zeroExtender && zeroArm) {
            return TerabytesOpModeState.HEADLESS_DRIVE;
        }

        return TerabytesOpModeState.INITIALIZE_TELEOP;
    }

    private TerabytesOpModeState evaluateManualControl() {

        if (gamepad1.a) {
            return TerabytesOpModeState.HEADLESS_DRIVE;
        } else if (gamepad1.x && hasPositionEstimate()) {
            initializeRadialDrivingMode();
            return TerabytesOpModeState.RADIAL_DRIVING_MODE;
        } else if (gamepad1.right_bumper && gamepad1.left_bumper) {
            return TerabytesOpModeState.HALT_OPMODE;
        }

        return TerabytesOpModeState.MANUAL_CONTROL;
    }

    private TerabytesOpModeState evaluateHeadlessDrivingMode() {
        if (gamepad1.x) {
            return TerabytesOpModeState.RADIAL_DRIVING_MODE;
        } else if (gamepad1.right_stick_button) {
            return TerabytesOpModeState.INITIALIZE_TELEOP;
        }

        double EXTENDER_GEAR_RATIO = 5.2d;
        double EXTENDER_TICS_PER_INCH = (EXTENDER_GEAR_RATIO * 28 / 0.8 / 2) * 2.54;
        double effectiveExtenderPositionInches = Math.max(0f, Math.min(2.0 + gamepad1.right_stick_x, EXTENDER_MAX_LENGTH_INCHES));
        extenderTickTarget = effectiveExtenderPositionInches * EXTENDER_TICS_PER_INCH;

        boolean slowMode = gamepad1.left_bumper;

        Pose2d driveInput = getScaledHeadlessDriverInput(gamepad1, allianceColor.OperatorHeadingOffset, slowMode);
        //drive.setWeightedDrivePower(driveInput);
        pincer.setPosition(pincerClosed ? PINCER_CLOSED : PINCER_OPEN);
        wrist.setPosition(WRIST_CENTER + (gamepad2.left_stick_x * WRIST_RANGE));

        int armTickTarget = 1000 + (int)(((1.0 + gamepad1.left_stick_x) / 2.0) * 5000);

        controlArmMotor(armLeft, armTickTarget);
        controlArmMotor(armRight, armTickTarget);

        if (a2Evaluator.evaluate()) {
            pincerClosed = !pincerClosed;
        }

        return TerabytesOpModeState.HEADLESS_DRIVE;
    }


    private TerabytesOpModeState evaluateCommandSequence() {
        if (commandSequence.isEmpty()) {
            TerabytesOpModeState _continuationState = continuationState;
            continuationState = null;
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            return _continuationState == null ? TerabytesOpModeState.MANUAL_CONTROL : _continuationState;
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
            logDesiredPositionTelemetry(currentCommand.DriveDirectToPose);
            drive.setWeightedDrivePower(
                    getPoseTargetAutoDriveControl(currentCommand.DriveDirectToPose));
        }

        boolean driveCompleted = currentCommand.DriveDirectToPose == null || isAtPoseTarget(currentCommand.DriveDirectToPose, currentCommand.SettleThresholdRatio);
        boolean settledRightNow = driveCompleted;

        boolean minTimeElapsed = currentCommandTime.milliseconds() > currentCommand.MinTimeMillis;
        boolean commandCompleted = settledRightNow && minTimeElapsed && currentCommandSettledTime.milliseconds() > currentCommand.SettleTimeMillis;
        boolean debugAdvance = !debugMode || gamepad1.a;
        if (commandCompleted && debugAdvance) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            drive.setWeightedDrivePower(new Pose2d());
            commandSequence.remove(0);
            currentCommand = null;
        } else if (!settledRightNow) {
            currentCommandSettledTime.reset();
        }

        return TerabytesOpModeState.COMMAND_SEQUENCE;
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

    private void evaluatePositioningSystems() {
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

    private void logPoseTelemetry(String caption, Pose2d pose2d) {
        telemetry.addData(caption, "X: %.1f Y: %.1f H: %.1f",
                pose2d.getX(),
                pose2d.getY(),
                Math.toDegrees(pose2d.getHeading()));
    }

    private void logDesiredPositionTelemetry(Pose2d pose2d) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        logPoseTelemetry("Pose Target: ", pose2d);
        logPoseTelemetry("Pose Estimate: ", poseEstimate);
        logPoseTelemetry("Pose Error: ", pose2d.minus(poseEstimate));
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
        setCommandSequence(TerabytesOpModeState.MANUAL_CONTROL, commands);
    }

    private void setCommandSequence(TerabytesOpModeState _continuationState, List<TerabytesCommand> commands) {
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
        drive.setWeightedDrivePower(new Pose2d());
        visionPortal.close();
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

    private TerabytesOpModeState evaluateRadialDrivingMode() {
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
        drive.setWeightedDrivePower(getPoseTargetAutoDriveControl(targetPose));

        // Exit condition to return to manual control
        if (gamepad1.a) {
            return TerabytesOpModeState.HEADLESS_DRIVE;
        }

        return TerabytesOpModeState.RADIAL_DRIVING_MODE;
    }

    private double fieldRadiusAtTheta(double theta) {
        double s = FIELD_SIDE_LENGTH; // Side length
        double k = 2 + squarenessParameter * 8; // k from 2 (circle) to 10 (square)

        double denom = Math.pow(Math.abs(Math.cos(theta)), k) + Math.pow(Math.abs(Math.sin(theta)), k);
        double radius = s / Math.pow(denom, 1.0 / k);

        return radius;
    }
}
