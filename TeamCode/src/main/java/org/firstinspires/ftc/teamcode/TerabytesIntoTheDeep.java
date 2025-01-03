package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.APRIL_TAG_RECOGNITION_YAW_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.FRONT_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.POSITION_ACQUIRED_INDICATE_MILLIS;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.POSITION_ACQUIRED_PULSE_MILLIS;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TerabytesIntoTheDeepConstants.TURN_GAIN;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    private static final class ServoInitStage {
        final double tilt, wrist, pincer;
        final long durationMs;
        ServoInitStage(double t, double w, double p, long d) {
            tilt = t; wrist = w; pincer = p; durationMs = d;
        }
    }

    private enum AppendageControlState {
        TUCKED,
        DEFENSIVE,
        COLLECTING,
        LOW_BASKET,
        HIGH_BASKET
    }

    private class AppendageControlTarget {
        public double armTickTarget;
        public double extenderTickTarget;
        public double tiltTarget;
        public double wristTarget;
        public double pincerTarget;

        public AppendageControlTarget(double a, double e, double t, double w, double p) {
            this.armTickTarget = a;
            this.extenderTickTarget = e;
            this.tiltTarget = t;
            this.wristTarget = w;
            this.pincerTarget = p;
        }
    }

    private final double GEAR_RATIO = 13.7d;
    private final double WORM_RATIO = 28.0d;
    private final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;
    private final double ARM_LEVEL_DEGREES_ABOVE_ZERO = 45; // TODO: Tune this to actual level up from min.
    private final double ARM_LEVEL_TICKS = ARM_LEVEL_DEGREES_ABOVE_ZERO * ARM_TICKS_PER_DEGREE;
    private final double ARM_COLLECT_MINIMUM_DEGREES = -22.5;
    private final double ARM_LEVEL_HEIGHT_INCHES = 15.5d;
    private final double ARM_COLLECT_HEIGHT_INCHES = 10d; // TODO: Tune this to actual desired
    private final double ARM_COLLECT_DEPTH_INCHES = ARM_LEVEL_HEIGHT_INCHES - ARM_COLLECT_HEIGHT_INCHES;
    private final double ARM_MAX_SETPOINT_SPEED_TICKS_PER_MILLI = (ARM_TICKS_PER_DEGREE * 30.0 / 1000.0);

    // TODO: tune extender parameters to get accurate ratio of tick per inch and no-extension length
    private final double EXTENDER_MIN_LENGTH_INCHES = 16d;
    private final double EXTENDER_GEAR_RATIO = 5.2d;
    private final double EXTENDER_TICS_PER_INCH = (EXTENDER_GEAR_RATIO * 28 / 0.8 / 2) * 2.54;
    private final double EXTENDER_MAX_LENGTH_INCHES = 14.5d;
    private final double EXTENDER_MAX_LENGTH_TICKS = EXTENDER_MAX_LENGTH_INCHES * EXTENDER_TICS_PER_INCH;
    private final double EXTENDER_SETPOINT_SPEED_TICKS_PER_MILLIS = (EXTENDER_TICS_PER_INCH * 1.5 / 1000.0);

    private final double TILT_ORIGIN = 0.0;
    private final double TILT_TICKS_PER_DEGREE = 1.0 / 270.0;
    private final double TILT_STRAIGHT = TILT_ORIGIN + (90 * TILT_TICKS_PER_DEGREE);
    private final double TILT_RANGE_DEGREES = 30.0;
    private final double TILT_RANGE = TILT_TICKS_PER_DEGREE * TILT_RANGE_DEGREES;
    private final double TILT_TUCKED = 0.925;
    private final double TILT_LOW_PROFILE = TILT_STRAIGHT;
    private final double TILT_PREGRAB = TILT_STRAIGHT / 2;
    private static final long GRAB_MOVE_DURATION_MS = 750;
    private static final long GRAB_HOLD_DURATION_MS = 150;

    private final double WRIST_CENTER = 0.95;
    private final double WRIST_RANGE = 0.25;
    private final double WRIST_TUCKED = WRIST_CENTER;

    private final double PINCER_CENTER = 0.575;
    private final double PINCER_OPEN = 0.5;
    private final double PINCER_CLOSED = 0.65;

    private final double NUDGE_MAX_INCHES_PER_MILLISECOND = 0.005;
    private final double NUDGE_MAX_RADIANS_PER_MILLISECOND = (Math.PI / 6) / 1000;
    private final double MAX_NUDGE_INCHES = 6;
    private final double MAX_NUDGE_RADIANS = Math.toRadians(15);

    private static final long GRAB_PHASE_1_MS = 750;
    private static final long GRAB_PHASE_2_MS = 150;
    private static final long GRAB_PHASE_3_MS = 750;

    private class AppendageControl {

        private AppendageControlState currentState;
        public final AppendageControlTarget target = new AppendageControlTarget(0, 0, TILT_ORIGIN, WRIST_CENTER, PINCER_CENTER);

        private double collectDistance = 0d;
        private boolean lowProfile = false;
        private ElapsedTime grabTimer = null;
        private ElapsedTime untuckTimer = null;

        public AppendageControl() {
            currentState = AppendageControlState.TUCKED;
        }

        public AppendageControlTarget evaluate() {
            if (untuckTimer != null) return evaluateUntucking();
            switch (currentState) {
                case TUCKED:     return evaluateTucked();
                case DEFENSIVE:  return evaluateDefensive();
                case COLLECTING: return evaluateCollecting();
                case LOW_BASKET: return evaluateLowBasket();
                case HIGH_BASKET:return evaluateHighBasket();
                default:         throw new IllegalArgumentException("Unexpected state: " + state);
            }
        }

        public void triggerUntuck() {
            untuckTimer = new ElapsedTime();
        }

        public void stopUntuck() {
            untuckTimer = null;
        }

        public void triggerGrab() {
            grabTimer = new ElapsedTime();
        }

        public void stopGrab() {
            grabTimer = null;
        }

        public void applyLowProfileCollection(boolean isLowProfile) {
            if (isLowProfile) stopGrab();
            lowProfile = isLowProfile;
        }

        public void setControlState(AppendageControlState state) {
            stopGrab();
            if (currentState == AppendageControlState.TUCKED) triggerUntuck();
            currentState = state;
        }

        public void applyCollectDistance(double collectDistance) {
            this.collectDistance = collectDistance;
        }

        private AppendageControlTarget evaluateTucked() {
            target.armTickTarget = 0;
            target.extenderTickTarget = 0;
            target.tiltTarget = TILT_TUCKED;
            target.wristTarget = WRIST_TUCKED;
            target.pincerTarget = PINCER_CLOSED;
            return target;
        }

        private AppendageControlTarget evaluateUntucking() {
            double t = untuckTimer.milliseconds();
            if (t < 2000) { // Arm moves toward level for 2s
                target.armTickTarget = ARM_LEVEL_TICKS;
                target.tiltTarget = TILT_TUCKED;
                target.wristTarget = WRIST_TUCKED;
                target.pincerTarget = PINCER_CLOSED;
            } else if (t < 2750) { // Wrist from tucked to center over 750ms
                double frac = (t - 2000) / 750.0;
                target.armTickTarget = ARM_LEVEL_TICKS;
                target.wristTarget = WRIST_TUCKED + frac * (WRIST_CENTER - WRIST_TUCKED);
                target.tiltTarget = TILT_TUCKED;
                target.pincerTarget = PINCER_CLOSED;
            } else if (t < 3500) { // Tilt from tucked to center over next 750ms
                double frac = (t - 2750) / 750.0;
                target.armTickTarget = ARM_LEVEL_TICKS;
                target.wristTarget = WRIST_CENTER;
                target.tiltTarget = TILT_TUCKED + frac * (TILT_ORIGIN - TILT_TUCKED);
                target.pincerTarget = PINCER_CLOSED;
            } else {
                stopUntuck();
            }
            return target;
        }

        private AppendageControlTarget evaluateDefensive() {
            double angle = 90, extInches = 0;
            target.armTickTarget = ARM_LEVEL_TICKS + angle * ARM_TICKS_PER_DEGREE;
            target.extenderTickTarget = EXTENDER_TICS_PER_INCH * extInches;
            target.tiltTarget = TILT_ORIGIN;
            target.wristTarget = WRIST_CENTER;
            target.pincerTarget = PINCER_CENTER;
            return target;
        }

        private AppendageControlTarget evaluateCollecting() {
            double clampedCollectDistance = Math.max(0, Math.min(1, collectDistance));
            double minimumAchievableDistance = EXTENDER_MIN_LENGTH_INCHES * Math.sin(Math.toRadians(ARM_COLLECT_MINIMUM_DEGREES));
            double maximumAchievableDistance = Math.sqrt((EXTENDER_MAX_LENGTH_INCHES * EXTENDER_MAX_LENGTH_INCHES) - (ARM_COLLECT_DEPTH_INCHES * ARM_COLLECT_DEPTH_INCHES));
            double desiredDistance = minimumAchievableDistance + (clampedCollectDistance * (maximumAchievableDistance - minimumAchievableDistance));
            double armAngle = -Math.toDegrees(Math.atan2(ARM_COLLECT_DEPTH_INCHES, desiredDistance));
            double desiredTotalLength = Math.sqrt((ARM_COLLECT_DEPTH_INCHES * ARM_COLLECT_DEPTH_INCHES) + (desiredDistance * desiredDistance));
            double desiredExtensionLength = EXTENDER_MIN_LENGTH_INCHES - desiredTotalLength;
            double extensionInches = Math.max(0, Math.min(EXTENDER_MIN_LENGTH_INCHES, desiredExtensionLength));

            // Now handle tilt & pincer:
            // 0) By default, if we are *not* grabbing, tilt is at “pregrab” and pincer is closed
            if (grabTimer == null) {
                target.tiltTarget = lowProfile ? TILT_LOW_PROFILE : TILT_PREGRAB;
                target.pincerTarget = PINCER_CLOSED;
            } else {
                double t = grabTimer.milliseconds();
                double p1End = GRAB_PHASE_1_MS;
                double p2End = GRAB_PHASE_1_MS + GRAB_PHASE_2_MS;
                double p3End = GRAB_PHASE_1_MS + GRAB_PHASE_2_MS + GRAB_PHASE_3_MS;

                if (t < p1End) {
                    // Phase 1: pincer opens, tilt moves from TILT_PREGRAB -> TILT_CENTER
                    double frac = t / GRAB_PHASE_1_MS;
                    double start = (lowProfile ? TILT_LOW_PROFILE : TILT_PREGRAB);
                    double end   = TILT_ORIGIN;
                    target.tiltTarget   = start + frac * (end - start);
                    target.pincerTarget = PINCER_OPEN;
                } else if (t < p2End) {
                    // Phase 2: pincer is closed, tilt stays at TILT_CENTER
                    target.tiltTarget   = TILT_ORIGIN;
                    target.pincerTarget = PINCER_CLOSED;
                } else if (t < p3End) {
                    // Phase 3: tilt goes from TILT_CENTER -> TILT_PREGRAB, pincer still closed
                    double frac = (t - p2End) / GRAB_PHASE_3_MS;
                    double start = TILT_ORIGIN;
                    double end   = (lowProfile ? TILT_LOW_PROFILE : TILT_PREGRAB);
                    target.tiltTarget   = start + frac * (end - start);
                    target.pincerTarget = PINCER_CLOSED;
                } else {
                    // Done with the grab sequence
                    stopGrab();
                    target.tiltTarget   = lowProfile ? TILT_LOW_PROFILE : TILT_PREGRAB;
                    target.pincerTarget = PINCER_CLOSED;
                }
            }

            // Keep wrist neutral in collecting
            target.wristTarget = WRIST_CENTER;
            return target;
        }

        private AppendageControlTarget evaluateLowBasket() {
            double angle = 105, extInches = 3;
            target.armTickTarget = ARM_LEVEL_TICKS + angle * ARM_TICKS_PER_DEGREE;
            target.extenderTickTarget = EXTENDER_TICS_PER_INCH * extInches;
            target.tiltTarget = TILT_ORIGIN;
            target.wristTarget = WRIST_CENTER;
            target.pincerTarget = PINCER_CENTER;
            return target;
        }

        private AppendageControlTarget evaluateHighBasket() {
            double angle = 105, extInches = 3;
            target.armTickTarget = ARM_LEVEL_TICKS + angle * ARM_TICKS_PER_DEGREE;
            target.extenderTickTarget = EXTENDER_TICS_PER_INCH * extInches;
            target.tiltTarget = TILT_ORIGIN;
            target.wristTarget = WRIST_CENTER;
            target.pincerTarget = PINCER_CENTER;
            return target;
        }
    }

    // Doubles as a maintenance mode where centered command can be easily found
    public final ServoInitStage[] SERVO_INIT_STAGES_AUTON = {
            new ServoInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN, 3750),
            new ServoInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000),
            new ServoInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN,3750),
            new ServoInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_CLOSED, 1000)
    };

    // Optimized for speed
    public final ServoInitStage[] SERVO_INIT_STAGES_TELEOP = {
            new ServoInitStage(TILT_TUCKED, WRIST_TUCKED, PINCER_OPEN,1000)
    };

    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

    // Basic state
    private final boolean debugMode;
    private final ElapsedTime loopTime = new ElapsedTime();
    private boolean isAutonomous = false;
    private IntoTheDeepOpModeState state;
    private ElapsedTime timeSinceInit;
    private ElapsedTime timeInState;
    private Pose2d latestPoseEstimate = null;

    // Basic gameplay state
    private final AllianceColor allianceColor;

    // April tag state
    private Pose2d lastAprilTagFieldPosition = null;
    private double lastAprilTagFieldPositionMillis = 0;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();

    // Command sequence state
    private final ArrayList<TerabytesCommand> commandSequence = new ArrayList<>();
    private TerabytesCommand currentCommand = null;
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
    private final OnActivatedEvaluator a1ActivatedEvaluator;
    private final OnActivatedEvaluator b1ActivatedEvaluator;
    private final OnActivatedEvaluator y1ActivatedEvaluator;
    private final OnActivatedEvaluator x1ActivatedEvaluator;
    private final OnActivatedEvaluator a2ActivatedEvaluator;

    // Sensing
    private final TouchSensor armMin;
    private final TouchSensor extenderMin;
    private final WebcamName frontCamera;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    // Indication
    private final DigitalChannel indicator1Red;
    private final DigitalChannel indicator1Green;

    // Appendage state
    private int servoInitStageIndex = 0;
    private ElapsedTime initStageTimer = new ElapsedTime();
    private AppendageControl appendageControl = null;

    private Pose2d submersibleNudge = new Pose2d();
    private Pose2d basketNudge = new Pose2d();

    public TerabytesIntoTheDeep(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.state = IntoTheDeepOpModeState.MANUAL_CONTROL;
        this.debugMode = debugMode;

        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
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

        // TODO: Setup the blinkin LEDs
        //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

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

        a2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
    }

    public Pose2d getLatestPoseEstimate() {
        return latestPoseEstimate;
    }

    private Pose2d driveInput = new Pose2d();
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

//        packet.put("lfc", leftFrontCurrent);
//        packet.put("lrc", leftRearCurrent);
//        packet.put("rfc", rightFrontCurrent);
//        packet.put("rrc", rightRearCurrent);
//
//        packet.put("lfp", leftFrontPower);
//        packet.put("lrp", leftRearPower);
//        packet.put("rfp", rightFrontPower);
//        packet.put("rrp", rightRearPower);
//
//        packet.put("lfv", leftFrontVelocity);
//        packet.put("lrv", leftRearVelocity);
//        packet.put("rfv", rightFrontVelocity);
//        packet.put("rrv", rightRearVelocity);

        // TODO: Get this reported into telemetry
        packet.put("currentState", state.toString());
        if (lastAprilTagFieldPosition != null) {
            packet.put("estimate-x", lastAprilTagFieldPosition.getX());
            packet.put("estimate-y", lastAprilTagFieldPosition.getY());
            packet.put("etimate-heading", lastAprilTagFieldPosition.getHeading());
        }

        packet.put("tilt", tilt.getPosition());
        packet.put("wrist", wrist.getPosition());
        packet.put("pincer", pincer.getPosition());
        packet.put("extenderPos", extender.getCurrentPosition());
        packet.put("extenderTarget", extender.getTargetPosition());
        packet.put("extenderMin", extenderMin.isPressed());
        packet.put("armMin", armMin.isPressed());

        if (appendageControl != null) {
            packet.put("armTickTarget", appendageControl.target.armTickTarget);
        }
        packet.put("ArmLeft-Power", armLeft.getPower());
        packet.put("ArmRight-Power", armRight.getPower());
        packet.put("ArmLeft-Position" + armLeft.getDeviceName(), armLeft.getCurrentPosition());
        packet.put("ArmRight-Position" + armRight.getDeviceName(), armRight.getCurrentPosition());
        packet.put("DriveInputX", driveInput.getX());
        packet.put("DriveInputY", driveInput.getY());
        packet.put("GamePad1X", gamepad1.left_stick_x);
        packet.put("GamePad1Y", gamepad1.left_stick_y);
        return packet;
    }

    public void autonomousInit(Pose2d startPose, TerabytesAutonomousPlan autonomousPlan) {
        timeSinceInit = new ElapsedTime();
        isAutonomous = true;
        drive.setPoseEstimate(startPose);
        activateFrontCameraProcessing();
        setCommandSequence(autonomousPlan.getCommandSequence(allianceColor));
    }

    public void teleopInit(Pose2d startPose) {
        timeSinceInit = new ElapsedTime();
        drive.setPoseEstimate(startPose == null ? new Pose2d() : startPose);
        activateFrontCameraProcessing();
    }

    public void initializeMechanicalBlocking() {
        // !!Do not touch controllers during mech init!!
        state = IntoTheDeepOpModeState.MANUAL_CONTROL;
        // Wait until we have tucked the appendage into the init position.
        while (appendageControl == null && evaluate()) {}
    }

    public void startup(IntoTheDeepOpModeState startupState) {
        timeInState = new ElapsedTime();
        state = startupState;
    }

    private void activateFrontCameraProcessing() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void clearNudge() {
        submersibleNudge = new Pose2d();
        basketNudge = new Pose2d();
    }

    private void evaluateAppendageInit() {
        ServoInitStage[] servoInitStages = isAutonomous ? SERVO_INIT_STAGES_AUTON : SERVO_INIT_STAGES_TELEOP;
        ServoInitStage stage = servoInitStages[servoInitStageIndex];
        tilt.setPosition(stage.tilt);
        wrist.setPosition(stage.wrist);
        pincer.setPosition(stage.pincer);

        if ((initStageTimer.milliseconds() > stage.durationMs) && (servoInitStageIndex < servoInitStages.length - 1)) {
            servoInitStageIndex++;
            initStageTimer.reset();
        }

        if (servoInitStageIndex == servoInitStages.length - 1) {
            // == Existing motor zeroing logic ==
            boolean zeroExtender = extenderMin.isPressed();
            if (zeroExtender) {
                extender.setPower(0.0);
                zeroExtender();
            } else {
                extender.setPower(-0.4);
            }

            boolean zeroArm = armMin.isPressed();
            if (zeroArm) {
                zeroArmMotors();
                armLeft.setPower(0);
                armRight.setPower(0);
            } else if (zeroExtender) {
                armLeft.setPower(-0.30);
                armRight.setPower(-0.30);
            }

            if (zeroExtender && zeroArm && appendageControl == null) {
                appendageControl = new AppendageControl();
            }
        }
    }

    private void evaluateAppendageControl() {
        AppendageControlTarget controlTarget = appendageControl.evaluate();
        controlArmMotors(controlTarget.armTickTarget);
        extender.setTargetPosition((int) controlTarget.extenderTickTarget);
        tilt.setPosition(controlTarget.tiltTarget);
        wrist.setPosition(controlTarget.wristTarget);
        pincer.setPosition(controlTarget.pincerTarget);
    }

    private void evaluateAppendageInitOrControl() {
        if (appendageControl != null) {
            evaluateAppendageControl();
        } else {
            evaluateAppendageInit();
        }
    }

    // Control loop.  Returns true iff the op-mode should continue running.
    public boolean evaluate() {
        double dt = loopTime.milliseconds();
        loopTime.reset();

        // Do the things we should do regardless of state
        // keep updating the drive and keep the machine alive
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        evaluateAppendageInitOrControl();
        evaluateIndicatorLights();

        // Determine whether there's been a state change
        // Run the state specific control logic
        IntoTheDeepOpModeState currentState = state;
        IntoTheDeepOpModeState nextState = evaluatePositioningSystems(currentState);

        switch (state) {
            case MANUAL_CONTROL:
                nextState = evaluateManualControl(dt);
                break;
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence();
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

    private void zeroExtender() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(20);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extender.setPower(0.8);
    }

    private void zeroArmMotors() {
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
        if(armPower < 0 && armMin.isPressed()) armPower = 0;
        armMotor.setPower(armPower);
    }

    private void controlArmMotors(double armTickTarget) {
        controlArmMotor(armTickTarget, leftArmControl, armLeft);
        controlArmMotor(armTickTarget, rightArmControl, armRight);
    }

    private IntoTheDeepOpModeState evaluateManualControl(double dtMillis) {
        // Example usage on gamepad2
        if (gamepad2.a) forceArmInit();

        if (appendageControl != null) {
            if (gamepad2.left_bumper) appendageControl.setControlState(AppendageControlState.COLLECTING);
            if (gamepad2.x || gamepad2.b) appendageControl.setControlState(AppendageControlState.DEFENSIVE);
            if (gamepad2.right_bumper) appendageControl.setControlState(AppendageControlState.HIGH_BASKET);
            if (gamepad2.a) appendageControl.triggerGrab();
            appendageControl.applyLowProfileCollection(gamepad2.y);
            appendageControl.applyCollectDistance(-gamepad2.right_trigger);
        }

        boolean slowMode = gamepad1.left_bumper;

        Pose2d subApproach = findNearestSubmersibleApproach();
        Pose2d basketApproach = IntoTheDeepPose.BASKET_APPROACH.getPose(allianceColor);

        if (gamepad1.a && subApproach != null) {
            submersibleNudge = accumulateNudge(submersibleNudge, gamepad2, true, dtMillis);
            Pose2d target = subApproach.plus(submersibleNudge);
            setDrivePower(getPoseTargetAutoDriveControl(target));
        } else if (gamepad1.y) {
            basketNudge = accumulateNudge(basketNudge, gamepad2, false, dtMillis);
            Pose2d target = basketApproach.plus(basketNudge);
            setDrivePower(getPoseTargetAutoDriveControl(target));
        } else {
            driveInput = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            setDrivePower(driveInput);
        }
        return IntoTheDeepOpModeState.MANUAL_CONTROL;
    }

    private Pose2d accumulateNudge(Pose2d prior, Gamepad gp, boolean submersibleMode, double dtMillis) {

        // TODO: Check direction here
        double fy = submersibleMode ? -gp.left_stick_y : gp.left_stick_y;
        double fx = gp.left_stick_x;
        double fh = -gp.right_stick_x;
        Pose2d updated = new Pose2d(
                prior.getX() + fx * NUDGE_MAX_INCHES_PER_MILLISECOND * dtMillis,
                prior.getY() + fy * NUDGE_MAX_INCHES_PER_MILLISECOND * dtMillis,
                prior.getHeading() + fh * NUDGE_MAX_RADIANS_PER_MILLISECOND * dtMillis
        );
        return limitNudge(updated, MAX_NUDGE_INCHES, MAX_NUDGE_RADIANS);
    }

    private Pose2d limitNudge(Pose2d p, double maxXY, double maxH) {
        double x = Range.clip(p.getX(), -maxXY, maxXY);
        double y = Range.clip(p.getY(), -maxXY, maxXY);
        double h = Range.clip(p.getHeading(), -maxH, maxH);
        return new Pose2d(x, y, h);
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

        double timeSincePositionSet = (timeSinceInit.milliseconds() - lastAprilTagFieldPositionMillis);
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
        double cameraDistanceOffset = FRONT_CAMERA_OFFSET_INCHES;
        double cameraAngleOffset = 0;

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
                lastAprilTagFieldPositionMillis = timeSinceInit.milliseconds();
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
}
