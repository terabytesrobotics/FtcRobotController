package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_YAW_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.FRONT_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_GAIN;

import android.util.ArrayMap;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Processors.SampleDetectVisionProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.EnumSet;

public class TerabytesIntoTheDeep {

    public static final double COLLECT_DISTANCE_ACCUMULATOR_SPEED_PER_MILLI = 1 / 1750.0;
    public static final double COLLECT_HEIGHT_ACCUMULATOR_SPEED_PER_MILLI = 1 / 2000.0;
    public static final double WRIST_ACCUMULATOR_SPEED_PER_MILLI = 1 / 500.0;

    public static final double GEAR_RATIO = 13.7d;
    public static final double WORM_RATIO = 28.0d;
    public static final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;
    public static final double ARM_LEVEL_DEGREES_ABOVE_ZERO = 59;
    public static final double ARM_LEVEL_TICKS = ARM_LEVEL_DEGREES_ABOVE_ZERO * ARM_TICKS_PER_DEGREE;
    public static final double ARM_AXLE_HEIGHT_INCHES = 15.5d;
    public static final double ARM_AXLE_OFFSET_FROM_ROBOT_CENTER_INCHES = 5.5; // TODO: Tune this to reality
    public static final double ARM_MIN_COLLECT_HEIGHT_INCHES = 5.75d;
    public static final double ARM_MAX_COLLECT_HEIGHT_INCHES = 13d;
    public static final double ARM_MIN_HEIGHT_WRIST_DETECT_INCHES = 10d;
    public static final double ARM_MAX_HEIGHT_WRIST_DETECT_INCHES = ARM_MAX_COLLECT_HEIGHT_INCHES;
    public static final double ARM_DEFENSIVE_ANGLE = 55.6;
    public static final double ARM_BASKET_ANGLE = 92;
    public static final double ARM_COLLECT_CLIP_ANGLE = -25;
    public static final double ARM_SCORE_CLIP_ANGLE = 36;
    public static final double ARM_PRE_HANG_ANGLE = 97.5;
    public static final double ARM_HANG_ANGLE = 20;
    public static final double EXTENDER_MIN_LENGTH_INCHES = 16d;
    public static final double EXTENDER_GEAR_RATIO = 5.2d;
    public static final double EXTENDER_TICKS_PER_INCH = (EXTENDER_GEAR_RATIO * 28 / 0.8 / 2) * 2.54;
    public static final double EXTENDER_MAX_EXTENSION_INCHES = 13.25d;
    public static final double EXTENDER_MAX_TOTAL_LENGTH = EXTENDER_MIN_LENGTH_INCHES + EXTENDER_MAX_EXTENSION_INCHES;
    public static final double EXTENDER_MAX_LENGTH_TICKS = EXTENDER_MAX_EXTENSION_INCHES * EXTENDER_TICKS_PER_INCH; //
    public static final double EXTENDER_DEFLECTION_RATIO = 1d / 12; // One inch per foot of extension
    public static final double EXTENDER_HANG = EXTENDER_MAX_EXTENSION_INCHES * 0.5;


    public static final double TILT_ORIGIN = 0.0;
    public static final double TILT_TICKS_PER_DEGREE = 1.0 / 270.0;
    public static final double TILT_STRAIGHT = TILT_ORIGIN + (90 * TILT_TICKS_PER_DEGREE);
    public static final double TILT_RANGE_DEGREES = 30.0;
    public static final double TILT_DOWN_RANGE = 40;
    public static final double TILT_RANGE = TILT_TICKS_PER_DEGREE * TILT_RANGE_DEGREES;
    public static final double TILT_TUCKED = 1.0;
    public static final double TILT_DUNK_RANGE = -0.25;
    public static final double TILT_LOW_PROFILE = TILT_STRAIGHT;
    public static final double TILT_PREGRAB = TILT_STRAIGHT / 2;

    public static final double WRIST_ORIGIN = 0.5;
    public static final double WRIST_RANGE = 0.35;
    public static final double WRIST_TUCKED = WRIST_ORIGIN;
    public static final double WRIST_DEGREES_TOTAL_RANGE = 300;
    public static final double WRIST_DEGREES_ALLOWABLE_HALF_RANGE = WRIST_RANGE * WRIST_DEGREES_TOTAL_RANGE;
    public static final double WRIST_DEGREES_HEADING_MAX = WRIST_RANGE * WRIST_DEGREES_TOTAL_RANGE;
    public static final double WRIST_DEGREES_HEADING_MIN = -WRIST_DEGREES_HEADING_MAX;

    public static final double PINCER_CENTER = 0.575;
    public static final double PINCER_OPEN = 0.5;
    public static final double PINCER_CLOSED = 0.65;

    // We don't yet support collecting at multiple distances in auton.
    public static final double AUTON_PRE_COLLECT_HEIGHT_SIGNAL = 0.75;
    public static final double AUTON_COLLECT_HEIGHT_SIGNAL = 0.1;
    public static final double AUTON_COLLECT_DISTANCE_SIGNAL = 0;
    public static final double AUTON_COLLECT_OFFSET_DISTANCE = 9.75;
    public static final double AUTON_COLLECT_WRIST_SIGNAL_ALIGNED = 0;
    public static final double AUTON_COLLECT_WRIST_SIGNAL_ACROSS = 0.9;

    // !! Must be run with arm up to be safe.  Helps calibrate servo positions for 10 seconds upon init. !!
    public final EndEffectorInitStage[] SERVO_INIT_STAGES_DEBUG = {
            new EndEffectorInitStage(TILT_ORIGIN, WRIST_ORIGIN, PINCER_CLOSED, 10000),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN, 3750),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN,3750),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000)
    };

    public final EndEffectorInitStage[] SERVO_INIT_STAGES_AUTON = {
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN, 3750),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN,3750),
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000)
    };

    // Optimized for speed
    public final EndEffectorInitStage[] SERVO_INIT_STAGES_TELEOP = {
            new EndEffectorInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN,1000)
    };

    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

    // Basic state
    private final boolean debugMode;
    private boolean isAutonomous = false;
    private IntoTheDeepOpModeState state;
    private final ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime timeSinceInit = new ElapsedTime();
    private ElapsedTime timeSinceStart = new ElapsedTime();
    private ElapsedTime timeInState = new ElapsedTime();
    private Pose2d latestPoseEstimate = null;

    // Actuation starting state
    private int armLTicksAtInit = 0;
    private int armRTicksAtInit = 0;
    private int extenderTicksAtInit = 0;

    // Basic gameplay state
    private final AllianceColor allianceColor;

    // April tag state
    private Pose2d lastAprilTagFieldPosition = null;
    private double lastAprilTagFieldPositionMillis = 0;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();

    // Command sequence state
    private final ArrayList<IntoTheDeepCommand> commandSequence = new ArrayList<>();
    private IntoTheDeepCommand currentCommand = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private IntoTheDeepOpModeState continuationState = null;

    // Actuation
    private final SampleMecanumDrive drive;
    private final PIDController leftArmControl = new PIDController(0.005, 0.00125, 0.0);
    private final PIDController rightArmControl = new PIDController(0.005, 0.00125, 0.0);
    private final DcMotorEx armLeft;
    private final DcMotorEx armRight;
    private final DcMotorEx extender;
    private final Servo tilt;
    private final Servo wrist;
    private final Servo pincer;

    // Controller
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

    // Sensing
    private final TouchSensor armMin;
    private final TouchSensor extenderMin;
    private final WebcamName frontCamera;
    private final WebcamName wristCamera;
    private final AprilTagProcessor aprilTagProcessor;
    private final ColorRangeSensor colorSensor;
    public final VisionPortal visionPortal;

    // NEW: Vision processor for sample detection
    private final SampleDetectVisionProcessor sampleDetectVisionProcessor;

    // Appendage state
    private int servoInitStageIndex = 0;
    private ElapsedTime initStageTimer = new ElapsedTime();
    private AppendageControl appendageControl = null;

    public TerabytesIntoTheDeep(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.state = IntoTheDeepOpModeState.MANUAL_CONTROL;
        this.debugMode = debugMode;

        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        wristCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager()
                .nameForSwitchableCamera(frontCamera, wristCamera);

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

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .addProcessor(sampleDetectVisionProcessor)
                .build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

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

        leftArmControl.setTolerance(20);
        rightArmControl.setTolerance(20);
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armLeft = hardwareMap.get(DcMotorEx.class, "armE0");
        armRight = hardwareMap.get(DcMotorEx.class, "armE3");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        tilt = hardwareMap.get(Servo.class, "tilt");
        pincer = hardwareMap.get(Servo.class, "pincer");
        wrist = hardwareMap.get(Servo.class, "wrist");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
    }

    public Pose2d getLatestPoseEstimate() {
        Pose2d latest = latestPoseEstimate;
        return latest == null ? new Pose2d() : latest;
    }

    public int getAppendageControlStateInteger() {
        return appendageControl == null ? -1 : appendageControl.currentState.ordinal();
    }

    public int getArmLTickPosition() {
        return armLeft.getCurrentPosition() + armLTicksAtInit;
    }

    public int getArmRTickPosition() {
        return armRight.getCurrentPosition() + armRTicksAtInit;
    }

    public int getExtenderTickPosition() {
        return extender.getCurrentPosition() + extenderTicksAtInit;
    }

    private Map<String, String> logData = new ArrayMap<>();
    public Map<String, String> getLogData() {
        logData.clear();

        // Put all the values we care about
        logData.put("blue", Integer.toString(colorSensor.blue()));
        logData.put("red", Integer.toString(colorSensor.red()));
        logData.put("green", Integer.toString(colorSensor.green()));
        logData.put("distance", Double.toString(colorSensor.getDistance(DistanceUnit.INCH)));

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
        packet.put("appendageState", appendageControl == null ? "UNINITIALIZED" : appendageControl.currentState);
        if (lastAprilTagFieldPosition != null) {
            packet.put("estimate-x", lastAprilTagFieldPosition.getX());
            packet.put("estimate-y", lastAprilTagFieldPosition.getY());
            packet.put("etimate-heading", lastAprilTagFieldPosition.getHeading());
        }

        packet.put("tilt", tilt.getPosition());
        packet.put("wrist", wrist.getPosition());
        packet.put("pincer", pincer.getPosition());

        packet.put("armMin", armMin.isPressed());
        packet.put("armLCurrentPosition", getArmLTickPosition());
        packet.put("armRCurrentPosition", getArmRTickPosition());
        packet.put("extenderMin", extenderMin.isPressed());
        packet.put("extenderCurrentPosition", getExtenderTickPosition());

        if (appendageControl != null) {
            int appendageControlArmLTickPosition = -getArmLTickPosition();
            int appendageControlArmRTickPosition = -getArmRTickPosition();
            int extenderTickPosition = getExtenderTickPosition();

            Double endEffectorHeight = appendageControl != null ?
                    appendageControl.getCurrentEndEffectorHeight(
                            appendageControlArmLTickPosition,
                            appendageControlArmRTickPosition,
                            extenderTickPosition) : null;

            double wristOffset = appendageControl.target.wristTarget - WRIST_ORIGIN;
            double wristHeading = (wristOffset / TerabytesIntoTheDeep.WRIST_RANGE) * WRIST_DEGREES_ALLOWABLE_HALF_RANGE;
            packet.put("wristHeadingOffset",  wristHeading);
            packet.put("armTickTarget", appendageControl.target.armTickTarget);
            packet.put("extenderTickTarget", appendageControl.target.extenderTickTarget);
            packet.put("endEffectorHeight", endEffectorHeight);
        }

        packet.put("armLTicksAtInit", armLTicksAtInit);
        packet.put("armRTicksAtInit", armRTicksAtInit);
        packet.put("extenderTicksAtInit", extenderTicksAtInit);

        packet.put("DriveInputX", driveInput.getX());
        packet.put("DriveInputY", driveInput.getY());
        packet.put("VisionProcessorAngle", sampleDetectVisionProcessor.detectedEllipseAngle);
        return packet;
    }

    public void autonomousInit(TerabytesAutonomousPlan autonomousPlan) {
        timeSinceInit.reset();
        isAutonomous = true;
        drive.setPoseEstimate(autonomousPlan.getStartingPose(allianceColor));
        setCommandSequence(autonomousPlan.getCommandSequence(allianceColor));
    }

    public void teleopInit(Pose2d startingPose) {
        timeSinceInit.reset();
        drive.setPoseEstimate(startingPose);
    }

    public void teleopInit(Pose2d startPose, AppendageControlState appendageControlState, int armLTickPosition, int armRTickPosition, int extenderTickPosition) {
        teleopInit(startPose);
        initExtenderMotor();
        initArmMotors();
        armLTicksAtInit = armLTickPosition;
        armRTicksAtInit = armRTickPosition;
        extenderTicksAtInit = extenderTickPosition;
        appendageControl = new AppendageControl(appendageControlState, isAutonomous);
    }

    public void initializeMechanicalBlocking() {
        // !!Do not touch controllers during mech init!!
        state = IntoTheDeepOpModeState.MANUAL_CONTROL;
        // Wait until we have tucked the appendage into the init position.
        while (appendageControl == null && evaluate()) {}
    }

    public void startup(IntoTheDeepOpModeState startupState) {
        timeSinceStart.reset();
        timeInState.reset();
        state = startupState;
    }

    private void evaluateAppendageInit() {
        EndEffectorInitStage[] servoInitStages = debugMode ? SERVO_INIT_STAGES_DEBUG : isAutonomous ? SERVO_INIT_STAGES_AUTON : SERVO_INIT_STAGES_TELEOP;
        EndEffectorInitStage stage = servoInitStages[servoInitStageIndex];
        boolean isFullyTuckedStage = servoInitStageIndex == servoInitStages.length - 1;
        boolean isStageFinished = initStageTimer.milliseconds() > stage.durationMs;

        tilt.setPosition(stage.tilt);
        wrist.setPosition(stage.wrist);
        pincer.setPosition(stage.pincer);

        if (isStageFinished && (servoInitStageIndex < servoInitStages.length - 1)) {
            servoInitStageIndex++;
            initStageTimer.reset();
        }

        if (isFullyTuckedStage) {
            boolean zeroExtender = extenderMin.isPressed();
            if (zeroExtender) {
                extender.setPower(0.0);
            } else {
                extender.setPower(-0.4);
            }

            boolean zeroArm = armMin.isPressed();
            if (zeroExtender && !zeroArm) {
                armLeft.setPower(0.35);
                armRight.setPower(0.35);
            } else {
                armLeft.setPower(0);
                armRight.setPower(0);
            }

            if (isStageFinished && zeroExtender && zeroArm && appendageControl == null) {
                initArmMotors();
                initExtenderMotor();
                armLTicksAtInit = 0;
                armRTicksAtInit = 0;
                extenderTicksAtInit = 0;
                appendageControl = new AppendageControl(AppendageControlState.TUCKED, isAutonomous);
            }
        }
    }

    private void evaluateAppendageControl() {
        int appendageControlArmLTickPosition = -getArmLTickPosition();
        int appendageControlArmRTickPosition = -getArmRTickPosition();
        int extenderTickPosition = getExtenderTickPosition();

        boolean isCollectingCameraDown = appendageControl != null && appendageControl.currentState == AppendageControlState.COLLECTING;
        Double endEffectorHeight = appendageControl != null ?
                appendageControl.getCurrentEndEffectorHeight(
                        appendageControlArmLTickPosition,
                        appendageControlArmRTickPosition,
                        extenderTickPosition) : null;
        boolean updateWristBasedOnVision = endEffectorHeight != null &&
                (endEffectorHeight < TerabytesIntoTheDeep.ARM_MAX_HEIGHT_WRIST_DETECT_INCHES && endEffectorHeight > TerabytesIntoTheDeep.ARM_MIN_HEIGHT_WRIST_DETECT_INCHES) &&
                !gamepad2.left_bumper;

        sampleDetectVisionProcessor.collectHeadingDegrees = appendageControl != null ? appendageControl.getCurrentWristHeadingDegrees() : null;

        if (isCollectingCameraDown && updateWristBasedOnVision) {
            Double ellipseAngle = sampleDetectVisionProcessor.detectedEllipseAngle;
            if (appendageControl != null && ellipseAngle != null) {
                appendageControl.updateVisionWristAdjustment(ellipseAngle);
            }
        }

        AppendageControlTarget controlTarget = appendageControl.evaluate(
                appendageControlArmLTickPosition,
                appendageControlArmRTickPosition,
                extenderTickPosition);
        double reversedTickTarget = -controlTarget.armTickTarget;
        controlArmMotor(reversedTickTarget - armLTicksAtInit, leftArmControl, armLeft);
        controlArmMotor(reversedTickTarget - armRTicksAtInit, rightArmControl, armRight);
        extender.setTargetPosition(((int) controlTarget.extenderTickTarget) - extenderTicksAtInit);
        tilt.setPosition(controlTarget.tiltTarget);
        wrist.setPosition(controlTarget.wristTarget);
        pincer.setPosition(controlTarget.pincerTarget);
    }

    private void evaluateSwitchCamera() {
        if (appendageControl == null || appendageControl.currentState != AppendageControlState.COLLECTING) {
            visionPortal.setActiveCamera(frontCamera);
        } else {
            visionPortal.setActiveCamera(wristCamera);
        }
    }

    private void evaluateAppendageInitOrControl() {
        if (appendageControl != null) {
            evaluateAppendageControl();
        } else {
            evaluateAppendageInit();
        }
    }

    public boolean evaluate() {
        double dt = loopTime.milliseconds();
        loopTime.reset();

        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        evaluateSwitchCamera();
        evaluateAppendageInitOrControl();
        evaluatePositioningSystems();

        boolean debugKill = debugMode &&
                ((gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a) ||
                        (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.a));

        IntoTheDeepOpModeState currentState = state;
        IntoTheDeepOpModeState nextState = currentState;
        switch (currentState) {
            case MANUAL_CONTROL:
                nextState = evaluateManualControl(dt);
                break;
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence();
                break;
            case STOPPED_UNTIL_END:
                armLeft.setMotorDisable();
                armRight.setMotorDisable();
                extender.setMotorDisable();
                setDrivePower(new Pose2d());
                break;
            default:
                break;
        }

        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        return state != IntoTheDeepOpModeState.HALT_OPMODE;
    }

    private Pose2d findNearestSubmersibleApproach() {
        Pose2d[] approaches = {
                IntoTheDeepPose.SUBMERSIBLE_APPROACH_ALLIANCE_SIDE.getPose(allianceColor),
                IntoTheDeepPose.SUBMERSIBLE_APPROACH_REAR_SIDE.getPose(allianceColor),
                IntoTheDeepPose.SUBMERSIBLE_APPROACH_OPPONENT_SIDE.getPose(allianceColor),
                IntoTheDeepPose.SUBMERSIBLE_APPROACH_AUDIENCE_SIDE.getPose(allianceColor)
        };
        if (latestPoseEstimate == null) return null;
        Pose2d nearest = approaches[0];
        double minDist = dist(latestPoseEstimate, nearest);
        for (Pose2d p : approaches) {
            double d = dist(latestPoseEstimate, p);
            if (d < minDist) {
                minDist = d;
                nearest = p;
            }
        }
        return nearest;
    }

    private double dist(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private void initExtenderMotor() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(20);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extender.setPower(0.8);
    }

    private void initArmMotors() {
        leftArmControl.reset();
        armLeft.setPower(0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArmControl.reset();
        armRight.setPower(0);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void forceArmInit() {
        appendageControl = null;
        servoInitStageIndex = 0;
        initStageTimer.reset();
    }

    private void controlArmMotor(double armTickTarget, PIDController controller, DcMotorEx armMotor) {
        double armPower = controller.calculate(armMotor.getCurrentPosition(), armTickTarget);
        if(Math.abs(armPower) < 0.02) armPower = 0;
        // Positive arm power is currently down.
        if(armPower > 0 && armMin.isPressed()) armPower = 0;
        armMotor.setPower(armPower);
    }

    private boolean isReadyToCollectAgain = true;

    private void setAppendageState(AppendageControlState state) {
        boolean isCollecting = state == AppendageControlState.COLLECTING;
        boolean isScoring = state == AppendageControlState.HIGH_BASKET ||
                state == AppendageControlState.LOW_BASKET ||
                state == AppendageControlState.SCORE_CLIP;
        if (isCollecting) {
            appendageControl.resetCollectParametersToDefault();
            appendageControl.setPincerOpen(isReadyToCollectAgain);
            isReadyToCollectAgain = false;
        } else if (isScoring) {
            isReadyToCollectAgain = true;
        }
        appendageControl.setControlState(state);
        visionPortal.setProcessorEnabled(sampleDetectVisionProcessor, isCollecting);
        visionPortal.setProcessorEnabled(aprilTagProcessor, !isCollecting);
    }

    private IntoTheDeepOpModeState evaluateManualControl(double dtMillis) {
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.a) {
            forceArmInit();
        }

        if (appendageControl != null) {
            if (a2ActivatedEvaluator.evaluate()) {
                if (appendageControl.currentState == AppendageControlState.DEFENSIVE ||
                        appendageControl.currentState == AppendageControlState.TUCKED ||
                        appendageControl.currentState == AppendageControlState.COLLECT_SAFE ||
                        appendageControl.currentState == AppendageControlState.PRE_HANG_1 ||
                        appendageControl.currentState == AppendageControlState.PRE_HANG_2 ||
                        appendageControl.currentState == AppendageControlState.HANG) {
                        setAppendageState(AppendageControlState.COLLECTING);
                } else if (appendageControl.currentState == AppendageControlState.HIGH_BASKET) {
                    setAppendageState(AppendageControlState.DEFENSIVE);
                } else if (appendageControl.currentState == AppendageControlState.COLLECT_CLIP ||
                        appendageControl.currentState == AppendageControlState.SCORE_CLIP) {
                    setAppendageState(AppendageControlState.COLLECTING);
                }
            } else if (y2ActivatedEvaluator.evaluate()) {
                if (appendageControl.currentState == AppendageControlState.COLLECTING ||
                        appendageControl.currentState == AppendageControlState.PRE_HANG_1 ||
                        appendageControl.currentState == AppendageControlState.PRE_HANG_2 ||
                        appendageControl.currentState == AppendageControlState.HANG) {
                    setAppendageState(AppendageControlState.COLLECT_SAFE);
                } else if (appendageControl.currentState == AppendageControlState.COLLECT_SAFE
                        || appendageControl.currentState == AppendageControlState.COLLECT_CLIP
                        || appendageControl.currentState == AppendageControlState.SCORE_CLIP) {
                    setAppendageState(AppendageControlState.DEFENSIVE);
                } else if (appendageControl.currentState == AppendageControlState.DEFENSIVE) {
                    setAppendageState(AppendageControlState.HIGH_BASKET);
                }
            }

            boolean lbActivated = lb1ActivatedEvaluator.evaluate();
            if (lbActivated) {
                // TODO: Shorten these long conditions into more simple local variable names.
                if (appendageControl.currentState == AppendageControlState.COLLECTING ||
                        appendageControl.currentState == AppendageControlState.DEFENSIVE ||
                        appendageControl.currentState == AppendageControlState.HIGH_BASKET ||
                        appendageControl.currentState == AppendageControlState.LOW_BASKET ||
                        appendageControl.currentState == AppendageControlState.COLLECT_SAFE ||
                        appendageControl.currentState == AppendageControlState.TUCKED) {
                    setAppendageState(AppendageControlState.PRE_HANG_1);
                } else if (appendageControl.currentState == AppendageControlState.PRE_HANG_1) {
                    setAppendageState(AppendageControlState.PRE_HANG_2);
                } else if (appendageControl.currentState == AppendageControlState.PRE_HANG_2) {
                    setAppendageState(AppendageControlState.HANG);
                }
            }


            if (x2ActivatedEvaluator.evaluate()) {
                if (appendageControl.currentState == AppendageControlState.COLLECT_CLIP) {
                    setAppendageState(AppendageControlState.SCORE_CLIP);
                } else {
                    setAppendageState(AppendageControlState.COLLECT_CLIP);
                }
            }

            if ((rb1ActivatedEvaluator.evaluate() && appendageControl.isBasketScoring()) || rb2ActivatedEvaluator.evaluate()) {
                appendageControl.togglePincer();
            }

            appendageControl.setDunkSignal(gamepad1.right_trigger + gamepad2.right_trigger);
            appendageControl.accumulateWristSignal(gamepad2.right_stick_x * WRIST_ACCUMULATOR_SPEED_PER_MILLI * dtMillis);
            appendageControl.applyTiltLevel(gamepad2.b || gamepad1.b);

            boolean isCollecting = appendageControl.currentState == AppendageControlState.COLLECTING;
            if (isCollecting) {


                double collectHeightSignal = -gamepad2.left_stick_y;
                double collectDistanceSignal = (gamepad1.right_trigger - gamepad1.left_trigger);
                int collectDistanceIncrements = y1ActivatedEvaluator.evaluate() ? 1 : a1ActivatedEvaluator.evaluate() ? -1 : 0;

                if (Math.abs(collectHeightSignal) > 0.025) {
                    appendageControl.accumulateCollectHeightSignal(collectHeightSignal * COLLECT_HEIGHT_ACCUMULATOR_SPEED_PER_MILLI * dtMillis);
                }

                if (Math.abs(collectDistanceSignal) > 0.2) {
                    appendageControl.accumulateCollectDistanceSignal(collectDistanceSignal * COLLECT_DISTANCE_ACCUMULATOR_SPEED_PER_MILLI * dtMillis);
                }

                // TODO: Deprecate?
                if (Math.abs(collectDistanceIncrements) > 0.025) {
                    appendageControl.incrementCollectDistance(collectDistanceIncrements);
                }
            }
        }

        boolean fastMode = gamepad1.left_bumper;
        boolean hasPositionEstimate = hasPositionEstimate();
        boolean isScoring = appendageControl != null && appendageControl.isBasketScoring();
        boolean wasScoring = appendageControl != null && (appendageControl.previousState == AppendageControlState.HIGH_BASKET || appendageControl.previousState == AppendageControlState.LOW_BASKET);
        boolean isDefensive = appendageControl != null && appendageControl.currentState == AppendageControlState.DEFENSIVE;

        double collectSideIsFront = isScoring ? -1d : 1d;
        Pose2d driverHeadlessInput = getScaledHeadlessDriverInput(gamepad1);

        if (gamepad1.dpad_right) {
            driveInput = new Pose2d(
                    0,
                    collectSideIsFront,
                    -gamepad1.right_stick_x);
        } else if (gamepad1.dpad_left) {
            driveInput = new Pose2d(
                    0,
                    -collectSideIsFront,
                    -gamepad1.right_stick_x);
        } else if (hasPositionEstimate && gamepad1.left_stick_button) {
            if (isScoring || (isDefensive && !wasScoring)) {
                driveInput = getAutoDriveToNetInput(isDefensive);
            } else {
                driveInput = new Pose2d();
            }
        }

        driveInput = driveInput.plus(driverHeadlessInput);

        if (fastMode) {
            driveInput = driveInput.div(1.5);
        } else {
            driveInput = driveInput.div(2.25);
        }

        setDrivePower(driveInput);
        return IntoTheDeepOpModeState.MANUAL_CONTROL;
    }

    private Pose2d getAutoDriveToNetInput(boolean reverseApproach) {
        if (!hasPositionEstimate()) return new Pose2d();

        Pose2d finalTarget = IntoTheDeepPose.HIGH_BASKET_SCORING_APPROACH.getPose(allianceColor);
        double errorX = Math.abs(finalTarget.getX() - latestPoseEstimate.getX());
        double errorY = Math.abs(finalTarget.getY() - latestPoseEstimate.getY());
        Pose2d targetForNow = new Pose2d(
                finalTarget.getX(),
                finalTarget.getY(),
                reverseApproach ? (finalTarget.getHeading() + Math.PI) : finalTarget.getHeading());
        if (errorX < 12.0 && errorY < 12.0) {
            targetForNow = finalTarget;
        } else if (errorX > errorY) {
            targetForNow = new Pose2d(
                    finalTarget.getX(),
                    finalTarget.getY(),
                    allianceColor.intoTheDeepNetApproachHeadingX());
        } else if (errorY > errorX) {
            targetForNow = new Pose2d(
                    finalTarget.getX(),
                    finalTarget.getY(),
                    allianceColor.intoTheDeepNetApproachHeadingY());
        }

        return getPoseTargetAutoDriveControl(targetForNow);
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad, double operatorHeadingOffset) {
        Vector2d inputFieldDirection = TerabytesHelpers.headlessLeftStickFieldDirection(gamepad, operatorHeadingOffset, latestPoseEstimate.getHeading());
        double scaledRobotX = inputFieldDirection.getX();
        double scaledRobotY = inputFieldDirection.getY();
        double scaledRotation = -gamepad.right_stick_x;
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad) {
        return getScaledHeadlessDriverInput(gamepad, allianceColor.OperatorHeadingOffset);
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
        }

        if (timeSinceStart.milliseconds() < currentCommand.WaitUntilElapsedMillis) {
            setDrivePower(new Pose2d());
            return IntoTheDeepOpModeState.COMMAND_SEQUENCE;
        }

        if (currentCommand.DriveToPose != null) {
            setDrivePower(
                    getPoseTargetAutoDriveControl(currentCommand.DriveToPose));
        }

        if (currentCommand.AppendageCommand != null && appendageControl != null) {
            setAppendageState(currentCommand.AppendageCommand.AppendageState);
            double dunkSignal = currentCommand.AppendageCommand.Dunk ? 1 : 0;
            double heightSignal = currentCommand.AppendageCommand.CollectHeightSignal != null ?
                    currentCommand.AppendageCommand.CollectHeightSignal :
                    0.5;
            double distanceSignal = currentCommand.AppendageCommand.CollectDistanceSignal != null ?
                    currentCommand.AppendageCommand.CollectDistanceSignal :
                    0.0;
            double wristSignal = currentCommand.AppendageCommand.WristSignal != null ?
                    currentCommand.AppendageCommand.WristSignal :
                    TerabytesIntoTheDeep.WRIST_ORIGIN;
            appendageControl.setHeightSignal(heightSignal);
            appendageControl.setDistanceSignal(distanceSignal);
            appendageControl.setWristSignal(wristSignal);
            appendageControl.setDunkSignal(dunkSignal);
            appendageControl.setPincerOpen(currentCommand.AppendageCommand.PincerOpen);
        }

        boolean driveCompleted = currentCommand.DriveToPose == null || isAtPoseTarget(currentCommand.DriveToPose, currentCommand.DriveSettleThresholdRatio);
        boolean appendageSettled = currentCommand.AppendageCommand == null || (appendageControl != null && appendageControl.isSettled());
        boolean settledRightNow = driveCompleted && appendageSettled;

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

            double translationVarianceThreshold = 1.0;
            double headingVarianceThreshold = Math.PI / 32;
            if (variancePose.getX() <= translationVarianceThreshold && variancePose.getY() <= translationVarianceThreshold && variancePose.getHeading() <= headingVarianceThreshold) {
                drive.setPoseEstimate(averagePose);
                lastAprilTagFieldPosition = averagePose;
                lastAprilTagFieldPositionMillis = timeSinceInit.milliseconds();
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

        double minPower = 0.2825;
        double minRotation = 0.6;
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

    private void setCommandSequence(List<IntoTheDeepCommand> commands) {
        setCommandSequence(IntoTheDeepOpModeState.STOPPED_UNTIL_END, commands);
    }

    private void setCommandSequence(IntoTheDeepOpModeState _continuationState, List<IntoTheDeepCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        continuationState = _continuationState;
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    public void shutDown() {
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setPower(0);

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setPower(0);

        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setPower(0);

        extender.setMotorDisable();
        armLeft.setMotorDisable();
        armRight.setMotorDisable();

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        drive.setDrivePower(new Pose2d(
                Math.abs(normalized.getX()) < 0.005 ? 0 : normalized.getX(),
                Math.abs(normalized.getY()) < 0.005 ? 0 : normalized.getY(),
                Math.abs(normalized.getHeading()) < 0.005 * Math.PI ? 0 : normalized.getHeading()
        ));
    }
}
