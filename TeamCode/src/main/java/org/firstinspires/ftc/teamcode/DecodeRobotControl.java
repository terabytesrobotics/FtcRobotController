package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_YAW_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_BLEND_HEADING_WEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_BLEND_TRANSLATION_WEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MAX_CORRECTION_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MAX_CORRECTION_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_VARIANCE_HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_CAMERA_LATERAL_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_CAMERA_HEIGHT_INCHES;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_CAMERA_OFFSET_INCHES;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MIN_QUEUE_SAMPLES;
import static org.firstinspires.ftc.teamcode.Constants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TURN_GAIN;

import android.util.ArrayMap;
import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Processors.SampleDetectVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.teamcode.HeadlessConfig;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

public class DecodeRobotControl {

    private static final double BALL_RADIUS_INCHES = 2.75;
    private static final double BALL_DIAMETER_INCHES = BALL_RADIUS_INCHES * 2;
    private static final double SHOOTER_WHEEL_RADIUS_INCHES = 2.0;
    private static final double SHOOTER_WHEEL_DIAMETER_INCHES = SHOOTER_WHEEL_RADIUS_INCHES * 2;
    private static final double FIELD_RIM_HEIGHT_INCHES = 39.0;
    private static final double RIM_CLEARANCE_INCHES = BALL_RADIUS_INCHES; // center clears rim by a radius
    private static final double TARGET_PLANE_HEIGHT_INCHES = FIELD_RIM_HEIGHT_INCHES + RIM_CLEARANCE_INCHES;
    private static final Vector2d RED_BASKET_POSITION_INCHES = new Vector2d(-72.0, 72.0);
    private static final Vector2d BLUE_BASKET_POSITION_INCHES = new Vector2d(-72.0, -72.0);
    private static final double SHOOTER_EXIT_ANGLE_RADIANS = Math.toRadians(50.0);
    // Ball exit height: bottom of ball at 13" above carpet -> center at 13" + radius.
    private static final double SHOOTER_EXIT_HEIGHT_INCHES = 13.0 + BALL_RADIUS_INCHES;
    // Shooter exit point relative to the robot center; +lateral is to the left, so right offset is negative.
    private static final double SHOOTER_FORWARD_OFFSET_INCHES = -2.0; // shooter exit sits 2" behind robot center
    private static final double SHOOTER_LATERAL_OFFSET_INCHES = -5.0; // shooter exit sits 5" to the right of robot center
    private static final double BALLISTIC_GRAVITY_IN_PER_S2 = 386.0886; // in/s^2
    // Simple top-spin model: extra downward load scales with spin rate; reduced to lighten long-shot drop.
    private static final double TOPSPIN_DROP_PER_RAD_PER_SEC = 0.002;

    private static final double SHOOTER_WHEEL_CIRCUMFERENCE_INCHES = Math.PI * SHOOTER_WHEEL_DIAMETER_INCHES;
    private static final double SHOOTER_CONTACT_ANGLE_RADIANS = Math.toRadians(130);
    // Arc length where the ball and wheel stay engaged; helps reason about acceleration distance.
    private static final double SHOOTER_CONTACT_ARC_LENGTH_INCHES = SHOOTER_WHEEL_RADIUS_INCHES * SHOOTER_CONTACT_ANGLE_RADIANS;
    // Efficiency factor baseline: exit velocity tends to trail the wheel surface speed because of slip/compression.
    private static final double SHOOTER_EXIT_VELOCITY_TRANSFER_BASE = 0.8;
    private static final double SHOOTER_TRANSFER_TRIM_RANGE = 0.1; // +/-10% via triggers
    private static final double SHOOTER_TRANSFER_MIN = 0.75;
    private static final double SHOOTER_TRANSFER_MAX = 1.05;
    private static final double SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND = 180.0;
    private static final double SHOOTER_MAX_EXIT_VELOCITY_INCHES_PER_SECOND = 450.0;
    private static final double SHOOTER_WHEEL_AXLE_HEIGHT_INCHES = 6.75;
    private static final double SHOOTER_WHEEL_COMPRESSION_INCHES = BALL_DIAMETER_INCHES + SHOOTER_WHEEL_RADIUS_INCHES - SHOOTER_WHEEL_AXLE_HEIGHT_INCHES;
    private static final double WHEEL_PPR = ((1+(46.0/17)) * 28);
    private static final double PRESENCE_PROXIMITY_THRESHOLD_INCHES = 1.85;
    private static final double GREEN_MATCH_THRESHOLD = 0.63;
    private static final double PURPLE_MATCH_THRESHOLD = 0.4;
    private static final double COLLECTOR_PRESENCE_ENTER_THRESHOLD = 0.125;
    private static final double COLLECTOR_PRESENCE_EXIT_THRESHOLD = 0.05;
    private static final double COLLECTOR_TRAVEL_DISTANCE_INCHES = 11.0;
    private static final double SLOT_SENSOR_PROXIMITY_THRESHOLD_INCHES = 1.25;
    private static final double SLOT_SENSOR_ENTER_THRESHOLD_GREEN = 0.15;
    private static final double SLOT_SENSOR_ENTER_THRESHOLD_PURPLE = 0.06;
    private static final double SLOT_SENSOR_EXIT_THRESHOLD = 0.03;
    private static final double SLOT_SENSOR_GREEN_PREFERENCE_MARGIN = 0.05;
    private static final double INTAKE_TICKS_PER_REV = 384.5; // encoder ticks per motor revolution
    private static final double INTAKE_ROLLER_RADIUS_INCHES = 2.5; // effective radius of the compliant roller
    private static final double INTAKE_COMPLIANCE_SLIP = 1.1; // >1 to account for band slip/compliance; tune on robot
    private static final double INTAKE_TICKS_PER_INCH = (INTAKE_TICKS_PER_REV / (2.0 * Math.PI * INTAKE_ROLLER_RADIUS_INCHES)) * INTAKE_COMPLIANCE_SLIP;
    private static final double SPIN_IN_TRANSIT_THRESHOLD = 0.003; // servo units; ~0.5 deg on a 5-turn servo
    private static final double SPIN_AT_TARGET_THRESHOLD = 0.002;
    private static final double KICKER_PULSE_SEC = 0.25;
    private static final double SHOOT_RECOVER_SEC = 0.1;
    private static final double SPIN_SETTLE_BEFORE_KICK_SEC = 0.25; // inflated to validate no premature kicks
    private static final double KICKER_SETTLE_AFTER_UNKICK_SEC = 0.15; // time to let the kicker clear the slot before motion

    private static final double GREEN_PRESENCE_THRESHOLD = 0.15;
    private static final double PURPLE_PRESENCE_THRESHOLD = 0.15;
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final double INTAKE_MOTOR_POWER = 0.6;
    private static final double INTAKE_POWER_SLEW_PER_SEC = 4.0; // limits bang-bang; full scale change in ~0.25s
    private static final int SPINDEXER_SLOT_COUNT = 3;
    private static final double KICKER_SERVO_RANGE_DEGREES = 270.0;
    private static final double KICKER_KICK_RANGE_DEGREES = 110.0; // expected travel for a full kick
    private static final double KICKER_KICK_RANGE = KICKER_KICK_RANGE_DEGREES / KICKER_SERVO_RANGE_DEGREES;
    // Start conservative; both positions are meant to be tuned on a real robot.
    private static final double KICKER_UNKICKED_POSITION = 0.05;
    private static final double KICKER_KICKED_POSITION = KICKER_UNKICKED_POSITION + KICKER_KICK_RANGE;
    // Rated 5-turn servo: 0-1 range maps to ~0-1800 degrees (tunable if real range differs).
    // Calibrated from field test: 2 slot moves were ~28 deg short (212 vs 240), so range is ~88.3% of nominal 1630.
    private static final double SPIN_SERVO_RANGE_DEGREES = (360.0 * 4.5) + 7.5;
    private static final double SPIN_SERVO_RANGE_TURNS = SPIN_SERVO_RANGE_DEGREES / 360.0;
    private static final double SPIN_SERVO_FULL_TURN = 1.0 / SPIN_SERVO_RANGE_TURNS;
    private static final double SPIN_SLOT_SPACING_DEGREES = 120.0;
    // Three slots 120 degrees apart -> converts degrees to servo position based on measured turn range.
    private static final double SPIN_SLOT_SPACING = (SPIN_SLOT_SPACING_DEGREES / 360.0) * SPIN_SERVO_FULL_TURN;
    // Tune this to align slot 0 with the collect pocket; leave at 0 to start.
    private static final double SPIN_BASE_POSITION_COLLECT_DEGREES = 22.85; // positive = clockwise nudge
    private static final double SPIN_BASE_POSITION_COLLECT = (SPIN_BASE_POSITION_COLLECT_DEGREES / 360.0) * SPIN_SERVO_FULL_TURN;
    // Offset from collect to shoot mode (in servo position units: 1.0 = 5 full turns = 1800 deg).
    // Approximately 2/5 of a turn between collect and shoot -> 144 degrees (applied in opposite direction).
    private static final double SPIN_MODE_OFFSET_DEGREES = 97.5;
    private static final double SPIN_MODE_OFFSET_SHOOT = (SPIN_MODE_OFFSET_DEGREES / 360.0) * SPIN_SERVO_FULL_TURN;
    private static final double SPIN_MAX_DEG_PER_SEC = 240.0;
    private static final double SPIN_MAX_POS_PER_SEC = (SPIN_MAX_DEG_PER_SEC / 360.0) * SPIN_SERVO_FULL_TURN; // 1.0 = full servo range
    private static final double SLOT_CHECK_SETTLE_SEC = 0.25;
    private static final double SLOT_CHECK_DWELL_SEC = 0.25;
    private static final int SLOT_CHECK_BURST_SAMPLES = 5;
    private static final double SLOT_CHECK_SAMPLE_SPACING_SEC = 0.02;
    private static final double SHOOT_AIM_HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);
    private static final double SHOOT_AIM_TURN_GAIN = 2.25; // scales heading error into rotation power while aiming
    // Teleop drive scaling: higher caps = more authority; fast mode bumps to full send.
    private static final double DRIVE_NORMAL_TRANSLATION_CAP = 0.85;
    private static final double DRIVE_FAST_TRANSLATION_CAP = 1.0;
    private static final double DRIVE_NORMAL_TURN_CAP = 0.85;
    private static final double DRIVE_FAST_TURN_CAP = 1.0;

    // Only trust the large field tags for localization.
    private static final int[] APRIL_TAG_ALLOWED_IDS = {20, 24};
    // Obelisk tags encode the green-ball position in the fixed 3-ball pattern.
    private static final int[] OBELISK_PATTERN_TAG_IDS = {21, 22, 33};
    // Obelisk faces +X on the -X perimeter; keep a tolerance so slight skew still counts.
    private static final double OBELISK_TARGET_HEADING_RADIANS = 0.0;
    private static final double OBELISK_HEADING_TOLERANCE_RADIANS = Math.toRadians(20.0);
    // Simple leave-autonomous starting/target definitions (base on red side; blue mirrors Y/heading).
    private static final double LEAVE_START_X = -60.0;
    private static final double LEAVE_START_Y = 50.0;
    private static final int LEAVE_RED_START_TAG_ID = 20;
    private static final int LEAVE_BLUE_START_TAG_ID = 24;
    private static final double LEAVE_TARGET_X = 12.0;
    private static final double LEAVE_TARGET_Y = 12.0;
    private static final double LEAVE_TARGET_HEADING = Math.toRadians(270.0);

    static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

    static double smoothstep(double edge0, double edge1, double x) {
        double t = clamp01((x - edge0) / (edge1 - edge0));
        return t * t * (3 - 2 * t);
    }

    static double belowThreshold(double x, double thresh, double softness) {
        return 1.0 - smoothstep(thresh, thresh + softness, x);
    }

    static double aboveThreshold(double x, double thresh, double softness) {
        return smoothstep(thresh, thresh + softness, x);
    }

    static double colorPresence(
            double proximityInches,
            double match,
            double proxThreshIn,
            double matchThresh,
            double proxSoftnessIn,
            double matchSoftness
    ) {
        double p = belowThreshold(proximityInches, proxThreshIn, proxSoftnessIn);
        double m = aboveThreshold(match, matchThresh, matchSoftness);
        return clamp01(p * m);
    }

    private final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getDecodeTagLibrary();
    private final boolean debugMode;
    private boolean isAutonomous = false;
    private OpModeState state;
    private final ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime timeSinceInit = new ElapsedTime();
    private ElapsedTime timeSinceStart = new ElapsedTime();
    private ElapsedTime timeInState = new ElapsedTime();
    private Pose2d latestPoseEstimate = new Pose2d(); // Null could be a good choice for unset
    private final AllianceColor allianceColor;
    private Pose2d lastAprilTagFieldPosition = null;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();
    private final ArrayList<OpModeCommand> commandSequence = new ArrayList<>();
    private OpModeCommand currentCommand = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private OpModeState continuationState = null;
    //private final SampleMecanumDrive drive;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final OnActivatedEvaluator rb1ActivatedEvaluator;
    private final OnActivatedEvaluator lb1ActivatedEvaluator;
    private final OnActivatedEvaluator a1ActivatedEvaluator;
    private final OnActivatedEvaluator liftToggleEvaluator;
    private final OnActivatedEvaluator x2ActivatedEvaluator;
    private final OnActivatedEvaluator b2ActivatedEvaluator;
    private final OnActivatedEvaluator y2ActivatedEvaluator;
    private final OnActivatedEvaluator a2ActivatedEvaluator;
    private final OnActivatedEvaluator rb2ActivatedEvaluator;
    private final DcMotorEx wheel;
    private final DcMotorEx intakeMotor;
    private final SampleMecanumDrive drive;
    private final Servo lift;
    private final WebcamName camera;
    private final AprilTagProcessor aprilTagProcessor;
    private final GoBildaPinpointDriver pinpoint;
    private final RevColorSensorV3 color1;
    private final RevColorSensorV3 color2;
    private final RevColorSensorV3 color3;
    private final IndicatorLed topLed;
    private final IndicatorLed midLed;
    private final IndicatorLed botLed;
    public final VisionPortal visionPortal;
    public final Servo spin;
    private final Servo kicker;
    private int spindexerSlot = 0; // 0-based physical pocket index
    private SpindexerMode spindexerMode = SpindexerMode.COLLECT;
    private double spindexerCanonicalTargetPosition = SPIN_BASE_POSITION_COLLECT;
    private double spindexerTargetPosition = SPIN_BASE_POSITION_COLLECT;
    private double spindexerCommandPosition = SPIN_BASE_POSITION_COLLECT;
    private final BallColor[] spindexerInventory = new BallColor[SPINDEXER_SLOT_COUNT]; // Slot-wise ball colors, filled from slot sensor.
    private boolean collectorPresenceLatched = false;
    private boolean collectorPresenceRisingEdge = false;
    private boolean collectorPresenceFallingEdge = false;
    private boolean collectorOverCapacity = false;
    private int collectorEstimatedBallCount = 0;
    private double intakePowerSmoothed = 0.0;
    private double lastCollectorPresence = 0.0;
    private double lastColor1ProximityInches = 0.0;
    private double lastColor2ProximityInches = 0.0;
    private double lastColor1GreenPresence = 0.0;
    private double lastColor1PurplePresence = 0.0;
    private double lastColor2GreenPresence = 0.0;
    private double lastColor2PurplePresence = 0.0;
    private double lastColor3ProximityInches = 0.0;
    private double lastColor3GreenPresence = 0.0;
    private double lastColor3PurplePresence = 0.0;
    private boolean color3PresenceLatched = false;
    private boolean spindexerInTransit = false;
    private IntakeState intakeState = IntakeState.FORWARD;
    private ShootCommandState shootCommandState = ShootCommandState.IDLE;
    private final ElapsedTime shootCommandTimer = new ElapsedTime();
    private final ElapsedTime spindexerSettleTimer = new ElapsedTime();
    private final ElapsedTime kickerSettleTimer = new ElapsedTime();
    private final ElapsedTime slotCheckPhaseTimer = new ElapsedTime();
    private final ElapsedTime slotCheckSampleTimer = new ElapsedTime();
    private SlotCheckPhase slotCheckPhase = SlotCheckPhase.IDLE;
    private int slotCheckActiveSlot = -1;
    private int slotCheckSamplesCollected = 0;
    private double slotCheckMaxGreenPresence = 0.0;
    private double slotCheckMaxPurplePresence = 0.0;
    private final double[] slotLastCheckTimeSeconds = new double[SPINDEXER_SLOT_COUNT];
    private final BallColor[] slotLastCheckColor = new BallColor[SPINDEXER_SLOT_COUNT];
    private final boolean[] slotLastCheckPresence = new boolean[SPINDEXER_SLOT_COUNT];
    private final int[] slotLastCheckSamples = new int[SPINDEXER_SLOT_COUNT];
    private final double[] slotLastCheckMaxGreen = new double[SPINDEXER_SLOT_COUNT];
    private final double[] slotLastCheckMaxPurple = new double[SPINDEXER_SLOT_COUNT];
    private final int[] slotEmptyStrikes = new int[SPINDEXER_SLOT_COUNT];
    private final ArrayDeque<Integer> slotCheckQueue = new ArrayDeque<>();
    private boolean shooterEnabled = true;
    private double shooterDesiredExitVelocityIps = 0.0;
    private double shooterDesiredWheelTicksPerSecond = 0.0;
    private double shooterLossTrim = 0.0;
    private double shooterTransferRatio = SHOOTER_EXIT_VELOCITY_TRANSFER_BASE;
    private boolean kickerKicked = false;
    private boolean kickerSettling = false;
    private ShotSolution lastShotSolution = null;
    private boolean lastShotBlockedByRim = false;
    private double lastDriveTranslationCap = DRIVE_NORMAL_TRANSLATION_CAP;
    private double lastDriveTurnCap = DRIVE_NORMAL_TURN_CAP;
    private ObeliskPattern obeliskPattern = ObeliskPattern.UNKNOWN;
    private final int[] obeliskPatternVotes = new int[ObeliskPattern.values().length];
    private int lastObeliskTagId = -1;
    private double lastObeliskTagHeading = Double.NaN;
    private double lastObeliskTagX = Double.NaN;
    // Keep the bulk A-button shooting logic available but opt-in; defaults off for teleop.
    private boolean bulkShootInputEnabled = false;
    private final ArrayDeque<ShotRequest> shotRequestQueue = new ArrayDeque<>();

    public DecodeRobotControl(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.state = OpModeState.MANUAL_CONTROL;
        this.debugMode = debugMode;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        color3 = hardwareMap.get(RevColorSensorV3.class, "color3");
        spin = hardwareMap.get(Servo.class, "spin");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Range.clip(KICKER_UNKICKED_POSITION, 0.0, 1.0));
        resetSpindexerInventory();
        initializeSlotCheckMetadata();
        initializeSpindexerToMidrange();
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLed = new IndicatorLed(hardwareMap, "topLedG", "topLedR");
        midLed = new IndicatorLed(hardwareMap, "midLedG", "midLedR");
        botLed = new IndicatorLed(hardwareMap, "botLedG", "botLedR");

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

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTagProcessor)
                .build();

        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        a1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        liftToggleEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_stick_button && gamepad2.right_stick_button);
        x2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        b2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        y2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        rb2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);
        a2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        lb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);

        drive = new SampleMecanumDrive(hardwareMap);

        lift = hardwareMap.get(Servo.class, "lift");

        configurePinpoint();

        drive.setLocalizer(new PinpointLocalizer(pinpoint));
        drive.setPoseEstimate(new Pose2d());

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
    }

    private Map<String, String> logData = new ArrayMap<>();
    public Map<String, String> getLogData() {
        logData.clear();

        return logData;
    }

    static double greenResonance(double r, double g, double b) {
        double eps = 1e-12;
        double ratio = g / (0.5*(r + b) + eps);     // >1 means green-dominant
        return ratio / (ratio + 1.0);               // maps (0..inf) -> (0..1)
    }

    static double purpleResonance(double r, double g, double b) {
        double eps = 1e-12;
        // Weight blue more than red to better favor purple samples over green spill/ambient.
        double ratio = ((0.3 * r) + (0.7 * b)) / (g + eps);    // >1 means magenta/purple-dominant
        return ratio / (ratio + 1.0);
    }

    private double sampleCollectorPresence() {
        int colorReadingMaxInt = 2 << 11;
        double red = (double) color1.red() / colorReadingMaxInt;
        double green = (double) color1.green() / colorReadingMaxInt;
        double blue = (double) color1.blue() / colorReadingMaxInt;
        double color1ProximityInches = color1.getDistance(DistanceUnit.INCH);
        double red2 = (double) color2.red() / colorReadingMaxInt;
        double green2 = (double) color2.green() / colorReadingMaxInt;
        double blue2 = (double) color2.blue() / colorReadingMaxInt;
        double color2ProximityInches = color2.getDistance(DistanceUnit.INCH);

        double greenMatch = greenResonance(red, green, blue);
        double purpleMatch = purpleResonance(red, green, blue);
        double greenMatch2 = greenResonance(red2, green2, blue2);
        double purpleMatch2 = purpleResonance(red2, green2, blue2);

        double proxSoft = 0.5;   // inches past threshold to fade out
        double matchSoft = 0.2;  // match past threshold to fade in

        double greenPresence = colorPresence(
                color1ProximityInches, greenMatch,
                PRESENCE_PROXIMITY_THRESHOLD_INCHES, GREEN_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );

        double purplePresence = colorPresence(
                color1ProximityInches, purpleMatch,
                PRESENCE_PROXIMITY_THRESHOLD_INCHES, PURPLE_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );

        double greenPresence2 = colorPresence(
                color2ProximityInches, greenMatch2,
                PRESENCE_PROXIMITY_THRESHOLD_INCHES, GREEN_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );

        double purplePresence2 = colorPresence(
                color2ProximityInches, purpleMatch2,
                PRESENCE_PROXIMITY_THRESHOLD_INCHES, PURPLE_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );

        double collectorPresence = Math.max(
                Math.max(greenPresence, purplePresence),
                Math.max(greenPresence2, purplePresence2)
        );

        lastCollectorPresence = collectorPresence;
        lastColor1ProximityInches = color1ProximityInches;
        lastColor2ProximityInches = color2ProximityInches;
        lastColor1GreenPresence = greenPresence;
        lastColor1PurplePresence = purplePresence;
        lastColor2GreenPresence = greenPresence2;
        lastColor2PurplePresence = purplePresence2;

        updateCollectorPresence(collectorPresence);
        return collectorPresence;
    }

    private Pose2d driveInput = new Pose2d();
    public TelemetryPacket getTelemetryPacket() {
        TelemetryPacket packet = new TelemetryPacket();

        double x = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getX();
        double y = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getY();
        double heading = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getHeading();

        double len = 12; // projection length
        double x2 = x + len * Math.cos(heading);
        double y2 = y + len * Math.sin(heading);

        packet.put("Color1ProximityInches", lastColor1ProximityInches);
        packet.put("Color1GreenPresence", lastColor1GreenPresence);
        packet.put("Color1PurplePresence", lastColor1PurplePresence);
        packet.put("Color2ProximityInches", lastColor2ProximityInches);
        packet.put("Color2GreenPresence", lastColor2GreenPresence);
        packet.put("Color2PurplePresence", lastColor2PurplePresence);
        packet.put("Color3ProximityInches", lastColor3ProximityInches);
        packet.put("Color3GreenPresence", lastColor3GreenPresence);
        packet.put("Color3PurplePresence", lastColor3PurplePresence);
        packet.put("Color3PresenceLatched", color3PresenceLatched);
        packet.put("CollectorPresence", lastCollectorPresence);
        packet.put("CollectorOverCapacity", collectorOverCapacity);
        packet.put("CollectorEstimatedBallCount", collectorEstimatedBallCount);
        packet.put("CollectorPresenceLatched", collectorPresenceLatched);
        packet.put("CollectorPresenceRising", collectorPresenceRisingEdge);
        packet.put("CollectorPresenceFalling", collectorPresenceFallingEdge);

        Canvas overlay = packet.fieldOverlay();
        Pose2d shooterPose = getShooterPoseEstimate();
        // Draw robot from the current pose estimate, and camera separately; tag pose is shown as the camera point.
        Pose2d poseForOverlay = latestPoseEstimate;
        double sx = poseForOverlay != null ? poseForOverlay.getX() : 0.0;
        double sy = poseForOverlay != null ? poseForOverlay.getY() : 0.0;
        double sheading = poseForOverlay != null ? poseForOverlay.getHeading() : 0.0;
        double shotLen = 12;
        double shx = sx + shotLen * Math.cos(sheading);
        double shy = sy + shotLen * Math.sin(sheading);

        // Predicted camera pose from robot pose + known offsets (for comparison against tag-derived camera pose).
        double predictedCamX = Double.NaN;
        double predictedCamY = Double.NaN;
        if (latestPoseEstimate != null) {
            double ch = latestPoseEstimate.getHeading();
            double offsetFieldX = (FRONT_CAMERA_OFFSET_INCHES * Math.cos(ch)) -
                    (FRONT_CAMERA_LATERAL_OFFSET_INCHES * Math.sin(ch));
            double offsetFieldY = (FRONT_CAMERA_OFFSET_INCHES * Math.sin(ch)) +
                    (FRONT_CAMERA_LATERAL_OFFSET_INCHES * Math.cos(ch));
            predictedCamX = latestPoseEstimate.getX() + offsetFieldX;
            predictedCamY = latestPoseEstimate.getY() + offsetFieldY;
        }

        if (!Double.isNaN(lastTagFieldX) && !Double.isNaN(lastTagFieldY)) {
            overlay.strokeCircle(lastTagFieldX, lastTagFieldY, 3);
        }
        if (!Double.isNaN(lastCameraFieldX) && !Double.isNaN(lastCameraFieldY)) {
            double camLen = 8;
            double camHx = lastCameraFieldX + camLen * Math.cos(lastCameraFieldHeading);
            double camHy = lastCameraFieldY + camLen * Math.sin(lastCameraFieldHeading);
            overlay.strokeCircle(lastCameraFieldX, lastCameraFieldY, 4)
                    .strokeLine(lastCameraFieldX, lastCameraFieldY, camHx, camHy);
        }
        if (!Double.isNaN(predictedCamX) && !Double.isNaN(predictedCamY)) {
            overlay.strokeCircle(predictedCamX, predictedCamY, 3);
        }

        overlay.fillCircle(sx, sy, 5)
                .strokeLine(sx, sy, shx, shy);
        drawSpindexerInventoryIcons(overlay, sx, sy);

        packet.put("loopTime", loopTime.milliseconds());
        packet.put("x", x);
        packet.put("y", y);
        packet.put("heading", heading);

        // TODO: Get this reported into telemetry
        packet.put("currentState", state.toString());
        if (lastAprilTagFieldPosition != null) {
            packet.put("estimate-x", lastAprilTagFieldPosition.getX());
            packet.put("estimate-y", lastAprilTagFieldPosition.getY());
            packet.put("estimate-heading", lastAprilTagFieldPosition.getHeading());
        }

        packet.put("lastDetectionYaw", lastDetectionYaw);
        packet.put("lastDetectionBearing", lastDetectionBearing);
        packet.put("lastDetectionElevation", lastDetectionElevation);
        packet.put("lastDetectionRange", lastDetectionRange);
        packet.put("lastTagFieldX", lastTagFieldX);
        packet.put("lastTagFieldY", lastTagFieldY);
        packet.put("lastTagFieldZ", lastTagFieldZ);
        packet.put("lastCameraFieldX", lastCameraFieldX);
        packet.put("lastCameraFieldY", lastCameraFieldY);
        packet.put("lastCameraFieldZ", lastCameraFieldZ);
        packet.put("lastCameraFieldHeading", lastCameraFieldHeading);
        packet.put("PredictedCameraX", predictedCamX);
        packet.put("PredictedCameraY", predictedCamY);
        if (!Double.isNaN(predictedCamX) && !Double.isNaN(lastCameraFieldX)) {
            packet.put("CameraXError", lastCameraFieldX - predictedCamX);
            packet.put("CameraYError", lastCameraFieldY - predictedCamY);
        }

        packet.put("G2_RSX", gamepad2.right_stick_x);
        packet.put("WheelCurrent", wheel.getCurrent(CurrentUnit.MILLIAMPS));
        double wheelVelocityTps = wheel.getVelocity();
        packet.put("WheelVelocity", wheelVelocityTps);
        packet.put("WheelVelocityInchesPerSecond", (wheelVelocityTps / WHEEL_PPR) * SHOOTER_WHEEL_CIRCUMFERENCE_INCHES);
        packet.put("WheelVelocityError", shooterDesiredWheelTicksPerSecond - wheelVelocityTps);
        packet.put("ShooterEnabled", shooterEnabled);
        packet.put("ShooterDesiredExitVelocityIps", shooterDesiredExitVelocityIps);
        packet.put("WheelDesiredRevPerSecond", shooterDesiredWheelTicksPerSecond / WHEEL_PPR);
        packet.put("WheelDesiredTickPerSecond", shooterDesiredWheelTicksPerSecond);
        packet.put("WheelEncoder", wheel.getCurrentPosition());
        packet.put("IntakeCurrent", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("IntakePower", intakeMotor.getPower());
        packet.put("IntakeState", intakeState.name());
        packet.put("Color1ProximityInches", lastColor1ProximityInches);
        packet.put("Color1GreenPresence", lastColor1GreenPresence);
        packet.put("Color1PurplePresence", lastColor1PurplePresence);
        packet.put("Color2ProximityInches", lastColor2ProximityInches);
        packet.put("Color2GreenPresence", lastColor2GreenPresence);
        packet.put("Color2PurplePresence", lastColor2PurplePresence);
        packet.put("CollectorPresence", lastCollectorPresence);
        packet.put("CollectorOverCapacity", collectorOverCapacity);
        packet.put("CollectorEstimatedBallCount", collectorEstimatedBallCount);
        packet.put("CollectorPresenceLatched", collectorPresenceLatched);
        packet.put("CollectorPresenceRising", collectorPresenceRisingEdge);
        packet.put("CollectorPresenceFalling", collectorPresenceFallingEdge);
        packet.put("ShootCommandState", shootCommandState.name());
        packet.put("DriveInputX", driveInput.getX());
        packet.put("DriveInputY", driveInput.getY());
        packet.put("DriveTranslationCap", lastDriveTranslationCap);
        packet.put("DriveTurnCap", lastDriveTurnCap);
        packet.put("SpindexerSlot", Math.floorMod(spindexerSlot, SPINDEXER_SLOT_COUNT) + 1); // human-friendly 1-based
        packet.put("SpindexerMode", spindexerMode == SpindexerMode.SHOOT ? "SHOOT" : "COLLECT");
        packet.put("SpindexerCanonicalTarget", spindexerCanonicalTargetPosition);
        packet.put("SpindexerTargetPosition", spindexerTargetPosition);
        packet.put("SpindexerCommandPosition", spindexerCommandPosition);
        packet.put("SpindexerServoPosition", spin.getPosition());
        packet.put("SpindexerDelta", spindexerTargetPosition - spindexerCommandPosition);
        packet.put("SpindexerInTransit", spindexerInTransit);
        packet.put("SlotCheckPhase", slotCheckPhase.name());
        packet.put("SlotCheckActiveSlot", slotCheckActiveSlot >= 0 ? slotCheckActiveSlot + 1 : -1);
        packet.put("SlotCheckQueueSize", slotCheckQueue.size());
        packet.put("SlotCheckSamples", slotCheckSamplesCollected);
        packet.put("SlotCheckMaxGreen", slotCheckMaxGreenPresence);
        packet.put("SlotCheckMaxPurple", slotCheckMaxPurplePresence);
        packet.put("ShotRequestQueueSize", shotRequestQueue.size());
        packet.put("ObeliskPattern", obeliskPattern.name());
        packet.put("ObeliskTagId", lastObeliskTagId);
        packet.put("ObeliskTagHeading", lastObeliskTagHeading);
        packet.put("ObeliskTagX", lastObeliskTagX);
        packet.put("ObeliskVotesGreenFirst", obeliskPatternVotes[ObeliskPattern.GREEN_FIRST.ordinal()]);
        packet.put("ObeliskVotesGreenMiddle", obeliskPatternVotes[ObeliskPattern.GREEN_MIDDLE.ordinal()]);
        packet.put("ObeliskVotesGreenLast", obeliskPatternVotes[ObeliskPattern.GREEN_LAST.ordinal()]);
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            packet.put("SpindexerSlot" + (i + 1) + "Color", spindexerInventory[i].name());
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckTimeSec", slotLastCheckTimeSeconds[i]);
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckPresence", slotLastCheckPresence[i]);
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckSamples", slotLastCheckSamples[i]);
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckMaxGreen", slotLastCheckMaxGreen[i]);
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckMaxPurple", slotLastCheckMaxPurple[i]);
            packet.put("SpindexerSlot" + (i + 1) + "LastCheckColor", slotLastCheckColor[i].name());
        }
        packet.put("KickerTargetPosition", kickerKicked ? KICKER_KICKED_POSITION : KICKER_UNKICKED_POSITION);
        packet.put("KickerServoPosition", kicker.getPosition());
        // Shot solution telemetry removed for clarity.
        packet.put("ShooterTransferRatio", shooterTransferRatio);
        packet.put("ShooterLossTrim", shooterLossTrim);
        packet.put("ShootAimError", getShooterHeadingError());

        packet.put("PinpointHeading", pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS));
        packet.put("PinpointX", pinpoint.getEncoderX());
        packet.put("PinpointY", pinpoint.getEncoderY());

        return packet;
    }

    public void autonomousInit(AutonomousPlan autonomousPlan) {
        timeSinceInit.reset();
        isAutonomous = true;
        Pose2d startPose = getStartPoseForPlan(autonomousPlan);
        drive.setPoseEstimate(startPose);
        lastAprilTagFieldPosition = startPose;
        setCommandSequence(buildAutonomousCommands(autonomousPlan));
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
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
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

        updateSpindexerIndicators();

        return state != OpModeState.HALT_OPMODE;
    }

    private double dist(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private boolean lifted = false;

    private double getDesiredExitVelocityIps() {
        double trigger = Range.clip(gamepad2.right_trigger, 0.0, 1.0);
        double scaledExitVelocity = Range.scale(
                trigger,
                0.0, 1.0,
                SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND,
                SHOOTER_MAX_EXIT_VELOCITY_INCHES_PER_SECOND);
        return Range.clip(
                scaledExitVelocity,
                SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND,
                SHOOTER_MAX_EXIT_VELOCITY_INCHES_PER_SECOND);
    }

    private double exitVelocityToWheelTicksPerSecond(double exitVelocityIps, double transferRatio) {
        double tangentialSpeedIps = exitVelocityIps / transferRatio;
        return (tangentialSpeedIps / SHOOTER_WHEEL_CIRCUMFERENCE_INCHES) * WHEEL_PPR;
    }

    private Vector2d getActiveBasketPosition() {
        return allianceColor == AllianceColor.RED ? RED_BASKET_POSITION_INCHES : BLUE_BASKET_POSITION_INCHES;
    }

    private Pose2d getShooterPoseEstimate() {
        Pose2d basePose = latestPoseEstimate != null ? latestPoseEstimate : lastAprilTagFieldPosition;
        if (basePose == null) return null;
        double heading = basePose.getHeading();
        double offsetX = (SHOOTER_FORWARD_OFFSET_INCHES * Math.cos(heading)) -
                (SHOOTER_LATERAL_OFFSET_INCHES * Math.sin(heading));
        double offsetY = (SHOOTER_FORWARD_OFFSET_INCHES * Math.sin(heading)) +
                (SHOOTER_LATERAL_OFFSET_INCHES * Math.cos(heading));
        return new Pose2d(
                basePose.getX() + offsetX,
                basePose.getY() + offsetY,
                heading);
    }

    private ShotSolution solveShotToActiveBasket(double transferRatio) {
        lastShotBlockedByRim = false;
        Pose2d shooterPose = getShooterPoseEstimate();
        if (shooterPose == null) return null;

        Vector2d basket = getActiveBasketPosition();
        double dx = basket.getX() - shooterPose.getX();
        double dy = basket.getY() - shooterPose.getY();
        double horizontalDistance = Math.hypot(dx, dy);
        if (horizontalDistance < 1e-3) return null;

        double headingToTarget = Math.atan2(dy, dx);
        double verticalDelta = TARGET_PLANE_HEIGHT_INCHES - SHOOTER_EXIT_HEIGHT_INCHES;
        double cosTheta = Math.cos(SHOOTER_EXIT_ANGLE_RADIANS);
        double sinTheta = Math.sin(SHOOTER_EXIT_ANGLE_RADIANS);
        double tanTheta = Math.tan(SHOOTER_EXIT_ANGLE_RADIANS);

        double verticalTerm = (horizontalDistance * tanTheta) - verticalDelta;
        if (verticalTerm <= 0.5) {
            lastShotBlockedByRim = true; // target too high/close for the fixed launch angle
            return null;
        }

        double effectiveGravity = BALLISTIC_GRAVITY_IN_PER_S2;
        double exitVelocityIps = SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND;
        double topSpinRadPerSec = 0.0;

        for (int i = 0; i < 3; i++) {
            double denom = 2.0 * cosTheta * cosTheta * verticalTerm;
            exitVelocityIps = Math.sqrt((effectiveGravity * horizontalDistance * horizontalDistance) / denom);
            double tangentialSpeedIps = exitVelocityIps / transferRatio;
            double surfaceRatio = tangentialSpeedIps / Math.max(1e-3, exitVelocityIps);
            double naturalRotations = SHOOTER_CONTACT_ARC_LENGTH_INCHES / (2 * Math.PI * BALL_RADIUS_INCHES);
            double avgLinear = Math.max(1e-3, 0.5 * (tangentialSpeedIps + exitVelocityIps));
            double contactTime = SHOOTER_CONTACT_ARC_LENGTH_INCHES / avgLinear;
            double rollSpinRadPerSec = (naturalRotations * 2 * Math.PI) / Math.max(1e-3, contactTime);
            double exitSpinRadPerSec = exitVelocityIps / BALL_RADIUS_INCHES;
            topSpinRadPerSec = 0.5 * (rollSpinRadPerSec + (exitSpinRadPerSec * surfaceRatio));
            double magnusMultiplier = 1.0 + Math.max(0.0, TOPSPIN_DROP_PER_RAD_PER_SEC * topSpinRadPerSec);
            effectiveGravity = BALLISTIC_GRAVITY_IN_PER_S2 * magnusMultiplier;
        }

        exitVelocityIps = Range.clip(
                exitVelocityIps,
                SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND,
                SHOOTER_MAX_EXIT_VELOCITY_INCHES_PER_SECOND);

        double planarSpeed = exitVelocityIps * cosTheta;
        double timeToPlane = solveTimeToHeight(effectiveGravity, exitVelocityIps, TARGET_PLANE_HEIGHT_INCHES);
        if (timeToPlane <= 0) {
            timeToPlane = horizontalDistance / Math.max(1e-3, planarSpeed);
        }

        double interceptX = shooterPose.getX() + (planarSpeed * Math.cos(headingToTarget) * timeToPlane);
        double interceptY = shooterPose.getY() + (planarSpeed * Math.sin(headingToTarget) * timeToPlane);

        ShotSolution solution = new ShotSolution();
        solution.exitVelocityIps = exitVelocityIps;
        solution.wheelTicksPerSecond = exitVelocityToWheelTicksPerSecond(exitVelocityIps, transferRatio);
        solution.effectiveGravity = effectiveGravity;
        solution.horizontalDistance = horizontalDistance;
        solution.verticalDelta = verticalDelta;
        solution.timeOfFlightSec = timeToPlane;
        solution.headingToTarget = headingToTarget;
        solution.interceptX = interceptX;
        solution.interceptY = interceptY;
        solution.topSpinRadPerSec = topSpinRadPerSec;
        solution.transferRatio = transferRatio;
        solution.tangentialSpeedIps = exitVelocityIps / transferRatio;
        return solution;
    }

    private double solveTimeToHeight(double effectiveGravity, double exitVelocityIps, double targetHeight) {
        double a = -0.5 * effectiveGravity;
        double b = exitVelocityIps * Math.sin(SHOOTER_EXIT_ANGLE_RADIANS);
        double c = SHOOTER_EXIT_HEIGHT_INCHES - targetHeight;
        double discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) return -1;

        double sqrtDisc = Math.sqrt(discriminant);
        double t1 = (-b + sqrtDisc) / (2 * a);
        double t2 = (-b - sqrtDisc) / (2 * a);

        double max = Math.max(t1, t2);
        double min = Math.min(t1, t2);
        if (max > 0) return max;
        if (min > 0) return min;
        return -1;
    }

    private static class SlotSensorSample {
        double proximityInches;
        double greenPresence;
        double purplePresence;
    }

    private static class ShotSolution {
        double exitVelocityIps;
        double wheelTicksPerSecond;
        double effectiveGravity;
        double horizontalDistance;
        double verticalDelta;
        double timeOfFlightSec;
        double headingToTarget;
        double interceptX;
        double interceptY;
        double topSpinRadPerSec;
        double transferRatio;
        double tangentialSpeedIps;
    }

    private static class ShotRequest {
        BallColor[] preferredColors;
        boolean allowAnyFallback;
    }

    private OpModeState evaluateManualControl(double dtMillis) {
        // Default shooter on; right bumper toggles off/on via the edge evaluator.
        if (rb2ActivatedEvaluator.evaluate()) {
            shooterEnabled = !shooterEnabled;
        }

        ShotSolution shotSolution = null;
        lastShotBlockedByRim = false;
        double trimInput = Range.clip(gamepad2.right_trigger - gamepad2.left_trigger, -1.0, 1.0);
        shooterLossTrim = Range.clip(
                trimInput * SHOOTER_TRANSFER_TRIM_RANGE,
                -SHOOTER_TRANSFER_TRIM_RANGE,
                SHOOTER_TRANSFER_TRIM_RANGE);
        shooterTransferRatio = Range.clip(
                SHOOTER_EXIT_VELOCITY_TRANSFER_BASE + shooterLossTrim,
                SHOOTER_TRANSFER_MIN,
                SHOOTER_TRANSFER_MAX);
        if (shooterEnabled) {
            shotSolution = solveShotToActiveBasket(shooterTransferRatio);
            if (shotSolution != null) {
                shooterDesiredExitVelocityIps = shotSolution.exitVelocityIps;
                shooterDesiredWheelTicksPerSecond = shotSolution.wheelTicksPerSecond;
            } else {
                shooterDesiredExitVelocityIps = getDesiredExitVelocityIps();
                shooterDesiredWheelTicksPerSecond = exitVelocityToWheelTicksPerSecond(shooterDesiredExitVelocityIps, shooterTransferRatio);
            }
            shooterDesiredWheelTicksPerSecond = exitVelocityToWheelTicksPerSecond(shooterDesiredExitVelocityIps, shooterTransferRatio);
            wheel.setVelocity(shooterDesiredWheelTicksPerSecond);
        } else {
            shooterDesiredExitVelocityIps = 0.0;
            shooterDesiredWheelTicksPerSecond = 0.0;
            wheel.setPower(0.0);
            lastShotBlockedByRim = false;
        }
        lastShotSolution = shotSolution;

        if (y2ActivatedEvaluator.evaluate()) {
            enqueueStalestSlotCheck();
        }

        boolean shotGreenRequest = a2ActivatedEvaluator.evaluate();
        boolean shotPurpleRequest = x2ActivatedEvaluator.evaluate();
        boolean shotAnyRequest = b2ActivatedEvaluator.evaluate();

        sampleCollectorPresence(); // keep telemetry updated; no longer drives intake control
        updateSlotCheckMachine();

        if (shotGreenRequest) {
            enqueueShotRequest(new BallColor[]{BallColor.GREEN}, false);
        }
        if (shotPurpleRequest) {
            enqueueShotRequest(new BallColor[]{BallColor.PURPLE}, false);
        }
        if (shotAnyRequest) {
            enqueueShotRequest(null, true);
        }

        serviceShotRequestQueue();

        updateShootCommand();

        updateKickerSettling();

        updateSpindexerPosition(dtMillis / 1000.0);
        spindexerInTransit = Math.abs(spindexerTargetPosition - spindexerCommandPosition) > SPIN_IN_TRANSIT_THRESHOLD;

        if (liftToggleEvaluator.evaluate()) {
            lifted = !lifted;
        }

        lift.setPosition(lifted ? 0.0 : 1.0);

        double kickerTarget = (spindexerMode == SpindexerMode.SHOOT && kickerKicked)
                ? KICKER_KICKED_POSITION
                : KICKER_UNKICKED_POSITION;
        kicker.setPosition(Range.clip(kickerTarget, 0.0, 1.0));

        boolean manualIntakeReverse = gamepad2.dpad_down;

        // Manual-only intake control: dpad down reverses, otherwise forward.
        if (manualIntakeReverse) {
            intakeState = IntakeState.REVERSE_REJECT;
        } else {
            intakeState = IntakeState.FORWARD;
        }

        double intakePowerTarget = 0.0;
        if (intakeState == IntakeState.FORWARD) {
            intakePowerTarget = INTAKE_MOTOR_POWER;
        } else if (intakeState == IntakeState.REVERSE_REJECT) {
            intakePowerTarget = -INTAKE_MOTOR_POWER;
        }
        double maxDelta = INTAKE_POWER_SLEW_PER_SEC * (dtMillis / 1000.0);
        double delta = Range.clip(intakePowerTarget - intakePowerSmoothed, -maxDelta, maxDelta);
        intakePowerSmoothed = Range.clip(intakePowerSmoothed + delta, -1.0, 1.0);
        intakeMotor.setPower(intakePowerSmoothed);

        boolean fastMode = gamepad1.left_bumper;

        double driverForwardHeading = HeadlessConfig.forwardHeadingRadians(allianceColor);
        driveInput = getHeadlessDriveInput(gamepad1, driverForwardHeading, latestPoseEstimate.getHeading());
        double translationCap = fastMode ? DRIVE_FAST_TRANSLATION_CAP : DRIVE_NORMAL_TRANSLATION_CAP;
        double turnCap = fastMode ? DRIVE_FAST_TURN_CAP : DRIVE_NORMAL_TURN_CAP;
        lastDriveTranslationCap = translationCap;
        lastDriveTurnCap = turnCap;
        driveInput = capDriveInput(driveInput, translationCap, turnCap);

        boolean autoAimActive = shootCommandState == ShootCommandState.AIMING || shootCommandState == ShootCommandState.ARMING;
        Pose2d driveCommand = autoAimActive ? getShootAimDrivePower() : driveInput;

        setDrivePower(driveCommand);
        return OpModeState.MANUAL_CONTROL;
    }

    private Pose2d getHeadlessDriveInput(Gamepad gamepad, double driverForwardHeading, double robotHeading) {
        // Robot-frame cardinal sanity check: D-pad drives pure cardinal without headless math.
        if (gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right) {
            final double dpadPower = 0.35; // gentle check, avoids full send during diagnostics
            double robotX = 0.0; // +X = robot forward, -X = robot back
            double robotY = 0.0; // +Y = robot left strafe, -Y = robot right strafe
            if (gamepad.dpad_up) {
                robotX = dpadPower;
            } else if (gamepad.dpad_down) {
                robotX = -dpadPower;
            } else if (gamepad.dpad_left) {
                robotY = dpadPower;
            } else if (gamepad.dpad_right) {
                robotY = -dpadPower;
            }
            return new Pose2d(robotX, robotY, 0.0);
        }

        Vector2d robotTranslation = Helpers.fieldRelativeLeftStick(gamepad, driverForwardHeading, robotHeading);
        // Right stick X: right = clockwise (negative in CCW-positive math)
        double rotation = -applySignedSquareDeadband(gamepad.right_stick_x, 0.02);
        return new Pose2d(robotTranslation.getX(), robotTranslation.getY(), rotation);
    }

    // Clamp translation magnitude and turn separately, then blend so the combined command still fits in the motor range.
    private Pose2d capDriveInput(Pose2d input, double translationCap, double turnCap) {
        double x = Range.clip(input.getX(), -translationCap, translationCap);
        double y = Range.clip(input.getY(), -translationCap, translationCap);
        double h = Range.clip(input.getHeading(), -turnCap, turnCap);

        double transMag = Math.hypot(x, y);
        if (transMag > translationCap && transMag > 1e-6) {
            double scale = translationCap / transMag;
            x *= scale;
            y *= scale;
            transMag = translationCap;
        }

        double combined = transMag + Math.abs(h);
        if (combined > 1.0) {
            double scale = 1.0 / combined;
            x *= scale;
            y *= scale;
            h *= scale;
        }

        return new Pose2d(x, y, h);
    }

    private Pose2d getScaledHeadlessDriverABInput(Gamepad gamepad, double driverForwardHeading) {
        Vector2d inputFieldDirection = Helpers.headlessABButtonFieldDirection(gamepad, driverForwardHeading, latestPoseEstimate.getHeading());
        double scaledRobotX = inputFieldDirection.getX();
        double scaledRobotY = inputFieldDirection.getY();
        double scaledRotation = -applySignedSquareDeadband(gamepad.right_stick_x, 0.02);
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private double applySignedSquareDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return Math.copySign(scaled * scaled, value);
    }

    private double computeCanonicalSpindexerPosition(int slot, SpindexerMode mode) {
        double canonical = SPIN_BASE_POSITION_COLLECT + (slot * SPIN_SLOT_SPACING);
        if (mode == SpindexerMode.SHOOT) {
            canonical -= SPIN_MODE_OFFSET_SHOOT;
        }
        return canonical;
    }

    // Choose the nearest in-range position to minimize travel on a multi-turn servo.
    private double findNearestTargetInRange(double canonicalTarget, double currentPosition) {
        double bestTarget = Range.clip(canonicalTarget, 0.0, 1.0);
        double bestDistance = Math.abs(bestTarget - currentPosition);
        int maxTurns = (int) Math.ceil(1.0 / SPIN_SERVO_FULL_TURN);
        for (int k = -maxTurns; k <= maxTurns; k++) {
            double candidate = canonicalTarget + (k * SPIN_SERVO_FULL_TURN);
            if (candidate < 0.0 || candidate > 1.0) {
                continue;
            }
            double distance = Math.abs(candidate - currentPosition);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTarget = candidate;
            }
        }
        return bestTarget;
    }

    private void retargetSpindexer() {
        spindexerCanonicalTargetPosition = computeCanonicalSpindexerPosition(spindexerSlot, spindexerMode);
        spindexerTargetPosition = findNearestTargetInRange(spindexerCanonicalTargetPosition, spindexerCommandPosition);
    }

    private void initializeSpindexerToMidrange() {
        spindexerMode = SpindexerMode.COLLECT;
        spindexerSlot = 0;
        spindexerCanonicalTargetPosition = computeCanonicalSpindexerPosition(spindexerSlot, spindexerMode);
        double centeredTarget = findNearestTargetInRange(spindexerCanonicalTargetPosition, 0.5);
        spindexerTargetPosition = centeredTarget;
        spindexerCommandPosition = centeredTarget;
        spin.setPosition(spindexerCommandPosition);
    }

    private void setKickerKicked(boolean kicked) {
        boolean wasKicked = kickerKicked;
        kickerKicked = kicked;
        if (kickerKicked) {
            kickerSettling = false;
        } else if (wasKicked) {
            kickerSettling = true;
            kickerSettleTimer.reset();
        }
    }

    private void updateKickerSettling() {
        if (kickerSettling && kickerSettleTimer.seconds() >= KICKER_SETTLE_AFTER_UNKICK_SEC) {
            kickerSettling = false;
        }
    }

    private boolean isSpindexerMotionAllowed() {
        return !kickerKicked && !kickerSettling;
    }

    private void updateSpindexerPosition(double dtSeconds) {
        spindexerTargetPosition = Range.clip(spindexerTargetPosition, 0.0, 1.0);
        double maxStep = SPIN_MAX_POS_PER_SEC * dtSeconds;
        double error = spindexerTargetPosition - spindexerCommandPosition;
        double step = Range.clip(error, -maxStep, maxStep);
        double nextPos = Range.clip(spindexerCommandPosition + step, 0.0, 1.0);
        if (isSpindexerMotionAllowed()) { // do not move when kicker is engaged or settling
            spindexerCommandPosition = nextPos;
            spin.setPosition(spindexerCommandPosition);
        }
    }

    private void updateSpindexerPosition() {
        spindexerTargetPosition = Range.clip(spindexerTargetPosition, 0.0, 1.0);
        if (isSpindexerMotionAllowed()) { // do not move when kicker is engaged or settling
            spindexerCommandPosition = spindexerTargetPosition;
            spin.setPosition(spindexerCommandPosition);
        }
    }

    private void resetSpindexerInventory() {
        Arrays.fill(spindexerInventory, BallColor.EMPTY);
    }

    private BallColor getSlotColor(int slotIndex) {
        int boundedIndex = Math.floorMod(slotIndex, SPINDEXER_SLOT_COUNT);
        return spindexerInventory[boundedIndex];
    }

    private void setSlotColor(int slotIndex, BallColor color) {
        int boundedIndex = Math.floorMod(slotIndex, SPINDEXER_SLOT_COUNT);
        spindexerInventory[boundedIndex] = color == null ? BallColor.EMPTY : color;
    }

    private int getKnownBallCount() {
        int count = 0;
        for (BallColor c : spindexerInventory) {
            if (c != BallColor.EMPTY) {
                count++;
            }
        }
        return count;
    }

    // Fusion of intake sensors to track a ball entering the mouth; hysteresis reduces flicker.
    // A latched presence represents a ball in the throat; we flag over-capacity if that implies a fourth ball.
    private void updateCollectorPresence(double collectorPresence) {
        collectorPresenceRisingEdge = false;
        collectorPresenceFallingEdge = false;
        boolean wasLatched = collectorPresenceLatched;
        if (!collectorPresenceLatched && collectorPresence >= COLLECTOR_PRESENCE_ENTER_THRESHOLD) {
            collectorPresenceLatched = true;
        } else if (collectorPresenceLatched && collectorPresence <= COLLECTOR_PRESENCE_EXIT_THRESHOLD) {
            collectorPresenceLatched = false;
        }
        collectorPresenceRisingEdge = !wasLatched && collectorPresenceLatched;
        collectorPresenceFallingEdge = wasLatched && !collectorPresenceLatched;

        collectorEstimatedBallCount = getKnownBallCount() + (collectorPresenceLatched ? 1 : 0);
        collectorOverCapacity = collectorEstimatedBallCount > SPINDEXER_SLOT_COUNT;
    }

    private void initializeSlotCheckMetadata() {
        Arrays.fill(slotLastCheckTimeSeconds, Double.NEGATIVE_INFINITY);
        Arrays.fill(slotLastCheckColor, BallColor.EMPTY);
        Arrays.fill(slotLastCheckPresence, false);
        Arrays.fill(slotLastCheckSamples, 0);
        Arrays.fill(slotLastCheckMaxGreen, 0.0);
        Arrays.fill(slotLastCheckMaxPurple, 0.0);
        Arrays.fill(slotEmptyStrikes, 0);
    }

    private void resetSlotCheckState() {
        slotCheckPhase = SlotCheckPhase.IDLE;
        slotCheckActiveSlot = -1;
        slotCheckSamplesCollected = 0;
        slotCheckMaxGreenPresence = 0.0;
        slotCheckMaxPurplePresence = 0.0;
        slotCheckPhaseTimer.reset();
        slotCheckSampleTimer.reset();
    }

    private void enqueueStalestSlotCheck() {
        if (slotCheckQueue.size() >= SPINDEXER_SLOT_COUNT) {
            return; // cap to avoid unbounded queue; 3 slots max
        }
        int slot = findStalestSlot();
        slotCheckQueue.add(slot);
    }

    private int findStalestSlot() {
        double oldestTimestamp = Double.POSITIVE_INFINITY;
        int oldestIndex = 0;
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            double ts = slotLastCheckTimeSeconds[i];
            if (ts == Double.NEGATIVE_INFINITY) {
                return i; // never checked; highest priority
            }
            if (ts < oldestTimestamp) {
                oldestTimestamp = ts;
                oldestIndex = i;
            }
        }
        return oldestIndex;
    }

    private SlotSensorSample readSlotSensor() {
        int colorReadingMaxInt = 2 << 11;
        double red3 = (double) color3.red() / colorReadingMaxInt;
        double green3 = (double) color3.green() / colorReadingMaxInt;
        double blue3 = (double) color3.blue() / colorReadingMaxInt;
        double color3ProximityInches = color3.getDistance(DistanceUnit.INCH);

        double proxSoft = 0.5;
        double matchSoft = 0.2;
        double greenPresence3 = colorPresence(
                color3ProximityInches,
                greenResonance(red3, green3, blue3),
                SLOT_SENSOR_PROXIMITY_THRESHOLD_INCHES, GREEN_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );
        double purplePresence3 = colorPresence(
                color3ProximityInches,
                purpleResonance(red3, green3, blue3),
                SLOT_SENSOR_PROXIMITY_THRESHOLD_INCHES, PURPLE_MATCH_THRESHOLD,
                proxSoft, matchSoft
        );

        lastColor3ProximityInches = color3ProximityInches;
        lastColor3GreenPresence = greenPresence3;
        lastColor3PurplePresence = purplePresence3;

        SlotSensorSample sample = new SlotSensorSample();
        sample.proximityInches = color3ProximityInches;
        sample.greenPresence = greenPresence3;
        sample.purplePresence = purplePresence3;
        return sample;
    }

    private BallColor classifySlotColor(double greenPresence, double purplePresence) {
        boolean greenHit = greenPresence >= SLOT_SENSOR_ENTER_THRESHOLD_GREEN;
        boolean purpleHit = purplePresence >= SLOT_SENSOR_ENTER_THRESHOLD_PURPLE;
        double slotPresence = Math.max(greenPresence, purplePresence);
        if (slotPresence < SLOT_SENSOR_EXIT_THRESHOLD) {
            return BallColor.EMPTY;
        }
        // Presence detected; pick a color even if thresholds are weak.
        if (greenPresence >= purplePresence + SLOT_SENSOR_GREEN_PREFERENCE_MARGIN || (greenPresence >= SLOT_SENSOR_ENTER_THRESHOLD_GREEN && !purpleHit)) {
            return BallColor.GREEN;
        }
        if (purplePresence > greenPresence) {
            return BallColor.PURPLE;
        }
        // Tie-break toward green for our green/not-green strategy.
        return BallColor.GREEN;
    }

    private void startSlotCheck(int slotIndex) {
        slotCheckActiveSlot = Math.floorMod(slotIndex, SPINDEXER_SLOT_COUNT);
        slotCheckPhase = SlotCheckPhase.WAIT_SETTLE;
        slotCheckSamplesCollected = 0;
        slotCheckMaxGreenPresence = 0.0;
        slotCheckMaxPurplePresence = 0.0;
        slotCheckPhaseTimer.reset();
        slotCheckSampleTimer.reset();
        spindexerMode = SpindexerMode.COLLECT;
        spindexerSlot = slotCheckActiveSlot;
        retargetSpindexer();
    }

    private void completeSlotCheckBurst() {
        BallColor detected = classifySlotColor(slotCheckMaxGreenPresence, slotCheckMaxPurplePresence);
        recordSlotCheck(slotCheckActiveSlot, slotCheckSamplesCollected, slotCheckMaxGreenPresence, slotCheckMaxPurplePresence, detected);
        if (!slotCheckQueue.isEmpty() && slotCheckQueue.peek() == slotCheckActiveSlot) {
            slotCheckQueue.poll();
        }
        slotCheckPhase = SlotCheckPhase.DWELL;
        slotCheckPhaseTimer.reset();
    }

    private void recordSlotCheck(int slotIndex, int samples, double maxGreen, double maxPurple, BallColor detected) {
        double nowSeconds = timeSinceStart.seconds();
        slotLastCheckTimeSeconds[slotIndex] = nowSeconds;
        slotLastCheckSamples[slotIndex] = samples;
        slotLastCheckMaxGreen[slotIndex] = maxGreen;
        slotLastCheckMaxPurple[slotIndex] = maxPurple;
        slotLastCheckColor[slotIndex] = detected;
        slotLastCheckPresence[slotIndex] = detected != BallColor.EMPTY;

        BallColor previous = getSlotColor(slotIndex);
        BallColor resolved = detected;
        if (detected == BallColor.EMPTY && previous != BallColor.EMPTY) {
            slotEmptyStrikes[slotIndex] = Math.min(slotEmptyStrikes[slotIndex] + 1, 10);
            if (slotEmptyStrikes[slotIndex] < 2) {
                resolved = previous; // require two consecutive empties to clear a known ball
            }
        } else {
            slotEmptyStrikes[slotIndex] = 0;
        }
        setSlotColor(slotIndex, resolved);
        color3PresenceLatched = resolved != BallColor.EMPTY;

        boolean becameFilled = previous == BallColor.EMPTY && resolved != BallColor.EMPTY;
        if (detected == BallColor.GREEN || detected == BallColor.PURPLE) {
            registerCollectedBallWithColor(detected);
        } else if (becameFilled && slotCheckQueue.isEmpty()) {
            int nextEmpty = findNearestEmptySlot(slotIndex);
            if (nextEmpty != slotIndex && !kickerKicked) {
                spindexerSlot = nextEmpty;
                retargetSpindexer();
            }
        }
    }

    private int findNearestEmptySlot(int startSlot) {
        int bestSlot = startSlot;
        int bestDistance = SPINDEXER_SLOT_COUNT + 1;
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            if (spindexerInventory[i] == BallColor.EMPTY) {
                int forward = Math.floorMod(i - startSlot, SPINDEXER_SLOT_COUNT);
                int backward = Math.floorMod(startSlot - i, SPINDEXER_SLOT_COUNT);
                int distance = Math.min(forward, backward);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    private void updateSlotCheckMachine() {
        if (!shotRequestQueue.isEmpty()) {
            resetSlotCheckState();
            return; // prioritize shooting over slot checks
        }
        boolean geometryReady = shootCommandState == ShootCommandState.IDLE && !kickerKicked && !kickerSettling;
        if (!geometryReady) {
            resetSlotCheckState();
            return;
        }

        int currentSlot = Math.floorMod(spindexerSlot, SPINDEXER_SLOT_COUNT);
        int desiredSlot = !slotCheckQueue.isEmpty() ? slotCheckQueue.peek() : currentSlot;

        if (slotCheckPhase == SlotCheckPhase.IDLE || slotCheckActiveSlot != desiredSlot) {
            startSlotCheck(desiredSlot);
        }

        if (spindexerSlot != desiredSlot || spindexerMode != SpindexerMode.COLLECT) {
            spindexerMode = SpindexerMode.COLLECT;
            spindexerSlot = desiredSlot;
            retargetSpindexer();
        }

        boolean settled = isSpindexerSettled();
        switch (slotCheckPhase) {
            case WAIT_SETTLE:
                if (!settled) {
                    slotCheckPhaseTimer.reset();
                } else if (slotCheckPhaseTimer.seconds() >= SLOT_CHECK_SETTLE_SEC) {
                    slotCheckPhase = SlotCheckPhase.SAMPLING;
                    slotCheckSampleTimer.reset();
                    slotCheckSamplesCollected = 0;
                    slotCheckMaxGreenPresence = 0.0;
                    slotCheckMaxPurplePresence = 0.0;
                }
                break;
            case SAMPLING:
                if (!settled) {
                    startSlotCheck(desiredSlot);
                    break;
                }
                if (slotCheckSamplesCollected == 0 || slotCheckSampleTimer.seconds() >= SLOT_CHECK_SAMPLE_SPACING_SEC) {
                    SlotSensorSample sample = readSlotSensor();
                    slotCheckMaxGreenPresence = Math.max(slotCheckMaxGreenPresence, sample.greenPresence);
                    slotCheckMaxPurplePresence = Math.max(slotCheckMaxPurplePresence, sample.purplePresence);
                    slotCheckSamplesCollected++;
                    slotCheckSampleTimer.reset();
                }
                if (slotCheckSamplesCollected >= SLOT_CHECK_BURST_SAMPLES) {
                    completeSlotCheckBurst();
                }
                break;
            case DWELL:
                if (!settled) {
                    startSlotCheck(desiredSlot);
                } else if (slotCheckPhaseTimer.seconds() >= SLOT_CHECK_DWELL_SEC) {
                    slotCheckPhase = SlotCheckPhase.WAIT_SETTLE;
                    slotCheckPhaseTimer.reset();
                    slotCheckSamplesCollected = 0;
                    slotCheckMaxGreenPresence = 0.0;
                    slotCheckMaxPurplePresence = 0.0;
                }
                break;
            case IDLE:
            default:
                startSlotCheck(desiredSlot);
                break;
        }
    }

    private void enqueueShotRequest(BallColor[] preferredColors, boolean allowAnyFallback) {
        ShotRequest req = new ShotRequest();
        req.preferredColors = preferredColors;
        req.allowAnyFallback = allowAnyFallback;
        shotRequestQueue.add(req);
    }

    private void serviceShotRequestQueue() {
        if (shootCommandState != ShootCommandState.IDLE) {
            return;
        }
        while (!shotRequestQueue.isEmpty()) {
            ShotRequest req = shotRequestQueue.peek();
            boolean started = startShootCommand(req.preferredColors, req.allowAnyFallback);
            shotRequestQueue.poll();
            if (started) {
                break;
            }
        }
    }

    private double getShooterHeadingError() {
        Pose2d shooterPose = getShooterPoseEstimate();
        Vector2d basket = getActiveBasketPosition();
        if (shooterPose == null || basket == null) {
            return 0.0;
        }
        double desiredHeading = Math.atan2(
                basket.getY() - shooterPose.getY(),
                basket.getX() - shooterPose.getX());
        double robotHeading = latestPoseEstimate != null ? latestPoseEstimate.getHeading() : shooterPose.getHeading();
        return Angle.normDelta(desiredHeading - robotHeading);
    }

    private boolean startShootCommand() {
        return startShootCommand(null, true);
    }

    private boolean startShootCommand(BallColor[] preferredColors, boolean allowAnyFallback) {
        int filledSlot = findNextFilledSlot(spindexerSlot, preferredColors, allowAnyFallback);
        if (filledSlot == -1) {
            shootCommandState = ShootCommandState.IDLE;
            return false;
        }
        spindexerSlot = filledSlot;
        shootCommandState = ShootCommandState.AIMING;
        shootCommandTimer.reset();
        spindexerMode = SpindexerMode.SHOOT;
        retargetSpindexer();
        setKickerKicked(false);
        spindexerSettleTimer.reset();
        return true;
    }

    private void updateShootCommand() {
        switch (shootCommandState) {
            case IDLE:
                return;
            case AIMING:
                spindexerMode = SpindexerMode.SHOOT;
                retargetSpindexer();
                if (isShooterAimed()) {
                    shootCommandState = ShootCommandState.ARMING;
                    spindexerSettleTimer.reset();
                }
                break;
            case ARMING:
                spindexerMode = SpindexerMode.SHOOT;
                retargetSpindexer();
                if (isSpindexerSettled()) {
                    if (spindexerSettleTimer.seconds() >= SPIN_SETTLE_BEFORE_KICK_SEC) {
                        setKickerKicked(true);
                        shootCommandState = ShootCommandState.KICKING;
                        shootCommandTimer.reset();
                    }
                } else {
                    spindexerSettleTimer.reset();
                }
                break;
            case KICKING:
                if (shootCommandTimer.seconds() >= KICKER_PULSE_SEC) {
                    setKickerKicked(false);
                    setSlotColor(spindexerSlot, BallColor.EMPTY);
                    shootCommandState = ShootCommandState.RECOVERING;
                    shootCommandTimer.reset();
                    spindexerMode = SpindexerMode.COLLECT;
                    retargetSpindexer();
                }
                break;
            case RECOVERING:
                if (shootCommandTimer.seconds() >= SHOOT_RECOVER_SEC) {
                    shootCommandState = ShootCommandState.IDLE;
                }
                break;
        }
    }

    private boolean isSpindexerAtTarget() {
        return Math.abs(spindexerTargetPosition - spindexerCommandPosition) <= SPIN_AT_TARGET_THRESHOLD;
    }

    private boolean isSpindexerSettled() {
        double error = Math.abs(spindexerTargetPosition - spindexerCommandPosition);
        boolean inTransit = error > SPIN_IN_TRANSIT_THRESHOLD;
        return !inTransit && isSpindexerAtTarget();
    }

    private boolean isShooterAimed() {
        return Math.abs(getShooterHeadingError()) <= SHOOT_AIM_HEADING_TOLERANCE_RADIANS;
    }

    private BallColor[] getPatternShootOrder() {
        switch (obeliskPattern) {
            case GREEN_FIRST:
                return new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
            case GREEN_MIDDLE:
                return new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
            case GREEN_LAST:
                return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
            default:
                return null;
        }
    }

    private void registerCollectedBallWithColor(BallColor color) {
        if (color != BallColor.GREEN && color != BallColor.PURPLE) {
            return; // only register on a confident color hit
        }
        setSlotColor(spindexerSlot, color);
        int nextSlot = findNextEmptySlot(spindexerSlot);
        if (nextSlot != spindexerSlot && !kickerKicked && slotCheckQueue.isEmpty()) {
            spindexerSlot = nextSlot;
            retargetSpindexer();
        }
    }

    private int findNextEmptySlot(int startSlot) {
        for (int i = 1; i <= SPINDEXER_SLOT_COUNT; i++) {
            int candidate = Math.floorMod(startSlot + i, SPINDEXER_SLOT_COUNT);
            if (spindexerInventory[candidate] == BallColor.EMPTY) {
                return candidate;
            }
        }
        return startSlot;
    }

    private int findNextSlotWithColor(int startSlot, BallColor desiredColor) {
        if (desiredColor == null || desiredColor == BallColor.EMPTY) {
            return -1;
        }
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            int candidate = Math.floorMod(startSlot + i, SPINDEXER_SLOT_COUNT);
            if (spindexerInventory[candidate] == desiredColor) {
                return candidate;
            }
        }
        return -1;
    }

    private int findNextFilledSlot(int startSlot, BallColor[] preferredColors, boolean allowAnyFallback) {
        if (preferredColors != null) {
            for (BallColor preferred : preferredColors) {
                int candidate = findNextSlotWithColor(startSlot, preferred);
                if (candidate != -1) {
                    return candidate;
                }
            }
            if (!allowAnyFallback) {
                return -1;
            }
        }
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            int candidate = Math.floorMod(startSlot + i, SPINDEXER_SLOT_COUNT);
            if (spindexerInventory[candidate] != BallColor.EMPTY) {
                return candidate;
            }
        }
        return -1;
    }

    private int findNextFilledSlot(int startSlot) {
        return findNextFilledSlot(startSlot, null, true);
    }

    private void updateSpindexerIndicators() {
        topLed.setColor(getSlotColor(0));
        midLed.setColor(getSlotColor(1));
        botLed.setColor(getSlotColor(2));
    }

    private void drawSpindexerInventoryIcons(Canvas overlay, double originX, double originY) {
        double spacing = 5.0;
        double radius = 2.0;
        double startX = originX - spacing;
        double y = originY + 8.0; // small offset behind the robot icon
        for (int i = 0; i < SPINDEXER_SLOT_COUNT; i++) {
            String fill = ballColorToFill(spindexerInventory[i]);
            String stroke = "#000000";
            double cx = startX + (i * spacing);
            overlay.setFill(fill);
            overlay.setStroke(stroke);
            overlay.fillCircle(cx, y, radius)
                    .strokeCircle(cx, y, radius);
        }
    }

    private String ballColorToFill(BallColor color) {
        switch (color) {
            case EMPTY:
                return "#cccccc";
            case GREEN:
                return "#00cc44";
            case PURPLE:
                return "#7a2cff";
            default:
                return "#555555"; // unknown/empty
        }
    }

    private Pose2d getShootAimDrivePower() {
        double headingError = getShooterHeadingError();
        double rotation = Range.clip(headingError * SHOOT_AIM_TURN_GAIN, -1.0, 1.0);
        return new Pose2d(0.0, 0.0, rotation);
    }

    private enum SlotCheckPhase {
        IDLE,
        WAIT_SETTLE,
        SAMPLING,
        DWELL
    }

    private enum SpindexerMode {
        COLLECT,
        SHOOT
    }

    private enum IntakeState {
        FORWARD,
        REVERSE_REJECT
    }

    private enum ShootCommandState {
        IDLE,
        AIMING,
        ARMING,
        KICKING,
        RECOVERING
    }

    private enum BallColor {
        EMPTY,
        GREEN,
        PURPLE
    }

    private enum ObeliskPattern {
        UNKNOWN,
        GREEN_FIRST,
        GREEN_MIDDLE,
        GREEN_LAST
    }

    // REV digital indicator helper; tries common name patterns for red/green channels.
    private static class IndicatorLed {
        private final LED red;
        private final LED green;

        IndicatorLed(HardwareMap hardwareMap, String greenName, String redName) {
            this.green = hardwareMap.get(LED.class, greenName);
            this.red = hardwareMap.get(LED.class, redName);
        }

        void setColor(BallColor color) {
            boolean redOn = false;
            boolean greenOn = false;
            switch (color) {
                case GREEN:
                    greenOn = true;
                    break;
                case PURPLE:
                    redOn = true;
                    break;
                case EMPTY:
                default:
                    break;
            }
            setLed(green, greenOn);
            setLed(red, redOn);
        }

        private void setLed(LED led, boolean on) {
            if (led == null) return;
            // Indicators are active-low DIO: driving low turns the LED on.
            if (on) {
                led.off();
            } else {
                led.on();
            }
        }
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
        double cameraForwardOffset = FRONT_CAMERA_OFFSET_INCHES;
        double cameraLateralOffset = FRONT_CAMERA_LATERAL_OFFSET_INCHES;
        double cameraHeightOffset = FRONT_CAMERA_HEIGHT_INCHES;
        double cameraAngleOffset = 0;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                maybeUpdateObeliskPattern(detection);

                if (!isAllowedAprilTag(detection.id)) {
                    continue;
                }

                if (detection.ftcPose == null) {
                    continue;
                }

                if (detection.ftcPose.range > APRIL_TAG_RECOGNITION_MAX_RANGE ||
                        detection.ftcPose.range < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                        Math.abs(Math.toRadians(detection.ftcPose.bearing)) > APRIL_TAG_RECOGNITION_BEARING_THRESHOLD ||
                        Math.abs(Math.toRadians(detection.ftcPose.yaw)) > APRIL_TAG_RECOGNITION_YAW_THRESHOLD) {
                    continue;
                }

                Pose2d estimatedPose = calculateRobotPose(
                        detection,
                        cameraForwardOffset,
                        cameraLateralOffset,
                        cameraHeightOffset,
                        cameraAngleOffset);
                if (poseQueue.size() >= APRIL_TAG_QUEUE_CAPACITY) {
                    poseQueue.poll();
                }
                poseQueue.offer(estimatedPose);
            }
        }

        if (poseQueue.size() >= APRIL_TAG_MIN_QUEUE_SAMPLES) {
            Pose2d averagePose = calculateAveragePose(poseQueue);
            Pose2d variancePose = calculateVariancePose(poseQueue, averagePose);

            boolean varianceAcceptable = variancePose.getX() <= APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD &&
                    variancePose.getY() <= APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD &&
                    variancePose.getHeading() <= APRIL_TAG_VARIANCE_HEADING_THRESHOLD;

            if (varianceAcceptable) {
                Pose2d fusedPose = averagePose;
                Pose2d basePose = latestPoseEstimate;

                if (basePose != null) {
                    double dx = averagePose.getX() - basePose.getX();
                    double dy = averagePose.getY() - basePose.getY();
                    double headingDelta = Angle.normDelta(averagePose.getHeading() - basePose.getHeading());
                    double distanceDelta = Math.hypot(dx, dy);
                    double headingDeltaAbs = Math.abs(headingDelta);

                    boolean snapToTag = distanceDelta > APRIL_TAG_MAX_CORRECTION_DISTANCE ||
                            headingDeltaAbs > APRIL_TAG_MAX_CORRECTION_HEADING;

                    if (!snapToTag) {
                        double blendedX = basePose.getX() + (dx * APRIL_TAG_BLEND_TRANSLATION_WEIGHT);
                        double blendedY = basePose.getY() + (dy * APRIL_TAG_BLEND_TRANSLATION_WEIGHT);
                        double blendedHeading = Angle.norm(basePose.getHeading() + (headingDelta * APRIL_TAG_BLEND_HEADING_WEIGHT));
                        fusedPose = new Pose2d(blendedX, blendedY, blendedHeading);
                    }
                }

                drive.setPoseEstimate(fusedPose);
                lastAprilTagFieldPosition = fusedPose;
                // Keep the queue so we always have a short smoothing window; trim to capacity to bound latency.
                while (poseQueue.size() > APRIL_TAG_QUEUE_CAPACITY) {
                    poseQueue.poll();
                }
            }
        }
    }

    private boolean isAllowedAprilTag(int tagId) {
        for (int allowedId : APRIL_TAG_ALLOWED_IDS) {
            if (allowedId == tagId) return true;
        }
        return false;
    }

    private boolean isObeliskTag(int tagId) {
        for (int obeliskId : OBELISK_PATTERN_TAG_IDS) {
            if (obeliskId == tagId) return true;
        }
        return false;
    }

    private void recordObeliskVote(ObeliskPattern pattern) {
        if (pattern == null || pattern == ObeliskPattern.UNKNOWN) {
            return;
        }
        obeliskPatternVotes[pattern.ordinal()]++;
        ObeliskPattern leader = getObeliskLeader(obeliskPattern);
        if (leader != ObeliskPattern.UNKNOWN) {
            obeliskPattern = leader;
        }
    }

    private ObeliskPattern getObeliskLeader(ObeliskPattern current) {
        int bestVotes = 0;
        ObeliskPattern leader = current;
        for (ObeliskPattern pattern : ObeliskPattern.values()) {
            if (pattern == ObeliskPattern.UNKNOWN) {
                continue;
            }
            int votes = obeliskPatternVotes[pattern.ordinal()];
            if (votes > bestVotes || (votes == bestVotes && pattern == current)) {
                bestVotes = votes;
                leader = pattern;
            }
        }
        return leader;
    }

    private ObeliskPattern patternFromTagId(int tagId) {
        switch (tagId) {
            case 21:
                return ObeliskPattern.GREEN_FIRST;
            case 22:
                return ObeliskPattern.GREEN_MIDDLE;
            case 33:
                return ObeliskPattern.GREEN_LAST;
            default:
                return ObeliskPattern.UNKNOWN;
        }
    }

    private void maybeUpdateObeliskPattern(AprilTagDetection detection) {
        if (!isObeliskTag(detection.id)) {
            return;
        }
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(detection.id);
        if (tag == null || tag.fieldPosition == null) {
            return;
        }

        lastObeliskTagId = detection.id;
        lastObeliskTagHeading = getTagFieldHeading(detection.id);
        lastObeliskTagX = tag.fieldPosition.get(0);

        if (detection.ftcPose == null) {
            return;
        }

        if (lastObeliskTagX >= 0) {
            return; // must live on -X side
        }

        double headingError = Math.abs(Angle.normDelta(lastObeliskTagHeading - OBELISK_TARGET_HEADING_RADIANS));
        if (headingError > OBELISK_HEADING_TOLERANCE_RADIANS) {
            return; // not facing +X
        }

        if (detection.ftcPose.range > APRIL_TAG_RECOGNITION_MAX_RANGE ||
                detection.ftcPose.range < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                Math.abs(Math.toRadians(detection.ftcPose.bearing)) > APRIL_TAG_RECOGNITION_BEARING_THRESHOLD ||
                Math.abs(Math.toRadians(detection.ftcPose.yaw)) > APRIL_TAG_RECOGNITION_YAW_THRESHOLD) {
            return;
        }

        ObeliskPattern observedPattern = patternFromTagId(detection.id);
        if (observedPattern != ObeliskPattern.UNKNOWN) {
            recordObeliskVote(observedPattern);
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

    double lastDetectionYaw = 0.0;
    double lastDetectionBearing = 0.0;
    double lastDetectionElevation = 0.0;
    double lastDetectionRange = 0.0;
    double lastTagFieldX = Double.NaN;
    double lastTagFieldY = Double.NaN;
    double lastTagFieldZ = Double.NaN;
    double lastCameraFieldX = Double.NaN;
    double lastCameraFieldY = Double.NaN;
    double lastCameraFieldZ = Double.NaN;
    double lastCameraFieldHeading = Double.NaN;

    private Pose2d calculateRobotPose(
            AprilTagDetection detection,
            double cameraRobotForwardOffset,
            double cameraRobotLateralOffset,
            double cameraRobotHeightOffset,
            double cameraRobotHeadingOffset) {
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(detection.id);
        if (tag == null) return null;

        double yaw = Math.toRadians(detection.ftcPose.yaw);
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double elevation = Math.toRadians(detection.ftcPose.elevation);
        double range = detection.ftcPose.range;
        lastDetectionYaw = yaw;
        lastDetectionBearing = bearing;
        lastDetectionElevation = elevation;
        lastDetectionRange = range;

        double tagFieldHeading = getTagFieldHeading(detection.id);

        double horizontalRange = range * Math.cos(elevation);
        double verticalOffset = range * Math.sin(elevation);

        double tagToCameraHeading = Angle.norm(tagFieldHeading + bearing - yaw);
        double tagFieldX = tag.fieldPosition.get(0);
        double tagFieldY = tag.fieldPosition.get(1);
        double tagFieldZ = tag.fieldPosition.get(2);
        double cameraFieldX = tagFieldX + (horizontalRange * Math.cos(tagToCameraHeading));
        double cameraFieldY = tagFieldY + (horizontalRange * Math.sin(tagToCameraHeading));
        double cameraFieldZ = tagFieldZ + verticalOffset;
        // Camera heading in field frame: robot is looking at the tag, so flip 180 deg from the tag normal and apply observed yaw.
        double cameraFieldHeading = Angle.norm(
                tagFieldHeading + Math.PI + cameraRobotHeadingOffset - yaw);

        double offsetFieldX = (cameraRobotForwardOffset * Math.cos(cameraFieldHeading)) -
                (cameraRobotLateralOffset * Math.sin(cameraFieldHeading));
        double offsetFieldY = (cameraRobotForwardOffset * Math.sin(cameraFieldHeading)) +
                (cameraRobotLateralOffset * Math.cos(cameraFieldHeading));
        double offsetFieldZ = cameraRobotHeightOffset;

        double robotFieldX = cameraFieldX - offsetFieldX;
        double robotFieldY = cameraFieldY - offsetFieldY;
        double robotFieldZ = cameraFieldZ - offsetFieldZ;

        lastTagFieldX = tagFieldX;
        lastTagFieldY = tagFieldY;
        lastTagFieldZ = tagFieldZ;
        // For telemetry, anchor the camera overlay to the robot pose plus the rotated offset so it stays consistent in robot frame.
        lastCameraFieldX = robotFieldX + offsetFieldX;
        lastCameraFieldY = robotFieldY + offsetFieldY;
        lastCameraFieldZ = robotFieldZ + offsetFieldZ;
        lastCameraFieldHeading = cameraFieldHeading;

        return new Pose2d(robotFieldX, robotFieldY, cameraFieldHeading);
    }

    private double getTagFieldHeading(int tagId) {
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(tagId);
        if (tag == null || tag.fieldOrientation == null) {
            return 0;
        }

        // Derive the heading from the tag's field orientation (SDK quaternion).
        // The SDK quaternion uses +Z pointing out the back of the tag, so the visible face normal is -Z.
        // Project that face normal onto the field XY plane to get heading.
        double w = tag.fieldOrientation.w;
        double x = tag.fieldOrientation.x;
        double y = tag.fieldOrientation.y;
        double z = tag.fieldOrientation.z;

        double forwardX = -2.0 * ((x * z) + (y * w));
        double forwardY = -2.0 * ((y * z) - (x * w));

        if (forwardX == 0.0 && forwardY == 0.0) {
            return 0;
        }

        return Angle.norm(Math.atan2(forwardY, forwardX));
    }

    private Pose2d mirrorPoseForBlue(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Angle.norm(-pose.getHeading()));
    }

    private Pose2d getLeaveStartPose(AllianceColor alliance) {
        int tagId = alliance == AllianceColor.RED ? LEAVE_RED_START_TAG_ID : LEAVE_BLUE_START_TAG_ID;
        double heading = Angle.norm(getTagFieldHeading(tagId) + Math.PI);
        double y = alliance == AllianceColor.RED ? LEAVE_START_Y : -LEAVE_START_Y;
        return new Pose2d(LEAVE_START_X, y, heading);
    }

    private Pose2d getLeaveTargetPose(AllianceColor alliance) {
        Pose2d base = new Pose2d(LEAVE_TARGET_X, LEAVE_TARGET_Y, LEAVE_TARGET_HEADING);
        return alliance == AllianceColor.RED ? base : mirrorPoseForBlue(base);
    }

    private Pose2d getStartPoseForPlan(AutonomousPlan plan) {
        // One simple plan; alliance selection comes from the opmode.
        return getLeaveStartPose(allianceColor);
    }

    private List<OpModeCommand> buildAutonomousCommands(AutonomousPlan plan) {
        List<OpModeCommand> commands = new ArrayList<>();
        // Single-move leave: target depends on alliance.
        commands.add(OpModeCommand.driveDirectToPoseCommand(getLeaveTargetPose(allianceColor)));
        return commands;
    }

    private void setCommandSequence(List<OpModeCommand> commands) {
        setCommandSequence(OpModeState.STOPPED_UNTIL_END, commands);
    }

    private void setCommandSequence(OpModeState _continuationState, List<OpModeCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        continuationState = _continuationState;
    }

    // Optional opt-in for bulk A-button shooting; remains disabled by default for teleop.
    public void setBulkShootInputEnabled(boolean enabled) {
        bulkShootInputEnabled = enabled;
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    public void shutDown() {
        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrivePower(new Pose2d());
        visionPortal.close();
    }

    public void setDrivePower(Pose2d drivePower) {
        // Our wiring/orientation has translation flipped; invert X/Y so operator forward/left match physical forward/left.
        double x = -drivePower.getX();
        double y = -drivePower.getY();
        double h = drivePower.getHeading();

        // Preserve full diagonal magnitude by normalizing translation vector before mixing rotation.
        double transMag = Math.hypot(x, y);
        if (transMag > 1.0) {
            x /= transMag;
            y /= transMag;
            transMag = 1.0;
        }

        double combined = transMag + Math.abs(h);
        if (combined > 1.0) {
            double scale = 1.0 / combined;
            x *= scale;
            y *= scale;
            h *= scale;
        }

        drive.setDrivePower(new Pose2d(
                Math.abs(x) < 0.005 ? 0 : x,
                Math.abs(y) < 0.005 ? 0 : y,
                Math.abs(h) < 0.005 * Math.PI ? 0 : h
        ));
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        // X pod is 120 mm to the right (left-positive, so -120), Y pod centered front/back.
        pinpoint.setOffsets(-120, 0, DistanceUnit.MM);

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
