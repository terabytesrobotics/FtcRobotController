package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_QUEUE_CAPACITY;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_VARIANCE_HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_BLEND_HEADING_WEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_BLEND_TRANSLATION_WEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MAX_CORRECTION_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MAX_CORRECTION_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_BEARING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_RECOGNITION_MIN_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_SANITY_MAX_BEARING;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_SANITY_MAX_RANGE;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_TO_POSE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.APRIL_TAG_MIN_QUEUE_SAMPLES;
import static org.firstinspires.ftc.teamcode.Constants.TOROID_JOYSTICK_DEADBAND;
import static org.firstinspires.ftc.teamcode.Constants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.TURN_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TURN_GAIN;

import android.util.ArrayMap;
import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Processors.SampleDetectVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

public class DecodeRobotControl {

    private static final String TAG = "DecodeRobotControl";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int LIMELIGHT_APRILTAG_PIPELINE = 0;
    private static final double LIMELIGHT_STALE_THRESHOLD_MS = 250.0;
    private static final double METERS_TO_INCHES = 39.37007874015748;
    private static final long ODOMETRY_HISTORY_WINDOW_NS = 1_000_000_000L;
    private static final double BALL_RADIUS_INCHES = 2.75;
    private static final double BALL_DIAMETER_INCHES = BALL_RADIUS_INCHES * 2;
    // Measured shooter wheel radius (inches) for trajectory math.
    private static final double SHOOTER_WHEEL_RADIUS_INCHES = 2.362205;
    private static final double SHOOTER_WHEEL_DIAMETER_INCHES = SHOOTER_WHEEL_RADIUS_INCHES * 2;
    private static final double FIELD_RIM_HEIGHT_INCHES = 39.0;
    private static final double RIM_CLEARANCE_INCHES = BALL_RADIUS_INCHES; // center clears rim by a radius
    private static final double TARGET_PLANE_HEIGHT_INCHES = FIELD_RIM_HEIGHT_INCHES + RIM_CLEARANCE_INCHES;
    private static final Vector2d RED_BASKET_POSITION_INCHES = new Vector2d(-69.5, 62.5);
    private static final double SHOOTER_EXIT_ANGLE_RADIANS = Math.toRadians(50.0);
    // Ball exit height: bottom of ball at 13" above carpet -> center at 13" + radius.
    private static final double SHOOTER_EXIT_HEIGHT_INCHES = 13.0 + BALL_RADIUS_INCHES;
    // Shooter exit point relative to the robot center; +lateral is to the left, so right offset is negative.
    private static final double SHOOTER_FORWARD_OFFSET_INCHES = -2.0; // shooter exit sits 2" behind robot center
    private static final double SHOOTER_LATERAL_OFFSET_INCHES = -5.0; // shooter exit sits 5" to the right of robot center
    private static final double BALLISTIC_GRAVITY_IN_PER_S2 = 386.0886; // in/s^2
    // Simple backspin model: lift reduces effective gravity proportional to spin rate.
    private static final double BACKSPIN_LIFT_PER_RAD_PER_SEC = 0.002;

    private static final double SHOOTER_WHEEL_CIRCUMFERENCE_INCHES = Math.PI * SHOOTER_WHEEL_DIAMETER_INCHES;
    private static final double SHOOTER_CONTACT_ANGLE_RADIANS = Math.toRadians(130);
    private static final double SHOOTER_ARC_COMPRESSION_INCHES = 0.25;
    // Ball center rides at wheel radius + ball diameter, minus compression from the arc.
    private static final double SHOOTER_BALL_PATH_RADIUS_INCHES =
            SHOOTER_WHEEL_RADIUS_INCHES + BALL_DIAMETER_INCHES - SHOOTER_ARC_COMPRESSION_INCHES;
    // Arc length where the ball and wheel stay engaged; helps reason about acceleration distance.
    private static final double SHOOTER_CONTACT_ARC_LENGTH_INCHES =
            SHOOTER_BALL_PATH_RADIUS_INCHES * SHOOTER_CONTACT_ANGLE_RADIANS;
    private static final double SHOOTER_SPIN_GEOMETRY_RATIO =
            SHOOTER_WHEEL_RADIUS_INCHES / SHOOTER_BALL_PATH_RADIUS_INCHES;
    // Empirical efficiency for how much of the ideal spin makes it to the ball (slip/compliance losses).
    private static final double SHOOTER_SPIN_EFFICIENCY = 0.35;
    // Efficiency factor baseline: exit velocity tends to trail the wheel surface speed because of slip/compression.
    private static final double SHOOTER_EXIT_VELOCITY_TRANSFER_BASE = 0.775;
    private static final double SHOOTER_TRANSFER_CLICK_STEP = 0.0125; // 1.25% per click
    private static final int SHOOTER_TRANSFER_CLICK_LIMIT = 4;
    private static final double SHOOTER_TRANSFER_MIN = 0.75;
    private static final double SHOOTER_TRANSFER_MAX = 1.05;
    private static final double SHOOTER_MIN_EXIT_VELOCITY_INCHES_PER_SECOND = 180.0;
    private static final double SHOOTER_MAX_EXIT_VELOCITY_INCHES_PER_SECOND = 450.0;
    private static final double SHOOTER_REVERSE_UNJAM_TRIGGER_THRESHOLD = 0.9;
    private static final double SHOOTER_REVERSE_UNJAM_RPM = 120.0;
    private static final double SHOOTER_REVERSE_UNJAM_MAX_WHEEL_TPS = 250.0;
    private static final double SHOOTER_CLOSE_RANGE_BOOST = 0.05; // +5% close-range boost
    private static final double SHOOTER_CLOSE_RANGE_MAX_DISTANCE_INCHES = 9.5 * 12.0;
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

    private static final double GREEN_PRESENCE_THRESHOLD = 0.15;
    private static final double PURPLE_PRESENCE_THRESHOLD = 0.15;
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final double INTAKE_MOTOR_POWER = 0.6;
    private static final double INTAKE_POWER_SLEW_PER_SEC = 4.0; // limits bang-bang; full scale change in ~0.25s
    private static final double INTAKE_JAM_CURRENT_AMPS = 1.6;
    private static final double INTAKE_JAM_SPEED_TPS = 200.0;
    private static final double INTAKE_JAM_DETECT_SEC = 0.12;
    private static final double INTAKE_JAM_RELAX_MAX_SEC = 0.2;
    private static final double INTAKE_JAM_STOP_TPS = 60.0;
    private static final double INTAKE_JAM_REVERSE_POWER = 0.335;
    private static final double INTAKE_JAM_REVERSE_SEC = 0.12;
    private static final double INTAKE_JAM_COOLDOWN_SEC = 0.2;
    // Observe-only intake event detector tuned for softer grab / longer throat.
    private static final double INTAKE_OBS_ARM_MILLIS = 450.0;
    private static final double INTAKE_OBS_REFRACTORY_MILLIS = 400.0;
    private static final double INTAKE_OBS_CURRENT_SPIKE_AMPS = 0.12;
    private static final double INTAKE_OBS_VELOCITY_DROP_RATIO = 0.12;
    private static final double INTAKE_OBS_BASELINE_ALPHA = 0.08;
    private static final double INTAKE_OBS_BASELINE_ALPHA_WHILE_CANDIDATE = 0.01;
    private static final double INTAKE_OBS_REARM_CURRENT_MARGIN_AMPS = 0.05;
    private static final double INTAKE_OBS_REARM_VELOCITY_MARGIN_RATIO = 0.05;
    private static final double INTAKE_TOGGLE_HOLD_SEC = 5.0;
    private static final String TOROID_MOTOR_NAME = "coreHex";
    private static final String TOROID_SENSOR_NAME = "color1";
    private static final String PADDLE_LIMIT_SWITCH_NAME = "mag";
    private static final String TOROID_ZERO_FILE_NAME = "toroid_zero.txt";
    private static final long TOROID_ZERO_FRESHNESS_MILLIS = 30_000L;
    private static final double TOROID_TICKS_PER_REV = 288.0;
    private static final double TOROID_STEP_TICKS = TOROID_TICKS_PER_REV / 3.0; // 120 deg steps
    private static final double TOROID_POSITION_TOLERANCE_TICKS = 6.0;
    private static final double TOROID_TRANSIT_MAX_RPM = 71.291015625;
    private static final double TOROID_LIMIT_SEEK_RPM = 18.0;
    private static final double TOROID_AUTON_INIT_SEEK_RPM = 10.0;
    private static final double TOROID_AUTON_INIT_STAGE_RPM = 10.0;
    private static final double TOROID_AUTON_START_OFFSET_DEGREES = 250.0;
    private static final double TOROID_HOME_OFFSET_DEGREES = -70.0;
    private static final double TOROID_HOME_HOLD_TOLERANCE_TICKS = 6.0;
    private static final double TOROID_HOME_HOLD_MAX_RPM = 12.0;
    private static final double TOROID_HOME_HOLD_KP_RPM_PER_TICK = 0.25;
    private static final double TOROID_HOME_DIRECT_THRESHOLD_TICKS = TOROID_STEP_TICKS; // 120 deg (one slot)
    // Keep autonomous shoot speed at or below what teleop can command at full stick.
    private static final double TOROID_SHOOT_RPM = TOROID_TRANSIT_MAX_RPM;
    private static final double TOROID_SHOOT_REVOLUTIONS = 2.5;
    private static final double TOROID_SHOOT_TIMEOUT_MILLIS = 7500.0;
    private static final int TOROID_KNOWN_OPENING_SHOT_COUNT = 3;
    private static final double TOROID_SHOT_DETECT_ARM_MILLIS = 350.0;
    private static final double TOROID_SHOT_DETECT_REFRACTORY_MILLIS = 250.0;
    private static final double TOROID_SHOT_DETECT_CURRENT_SPIKE_AMPS = 0.35;
    private static final double TOROID_SHOT_DETECT_VELOCITY_DROP_RATIO = 0.18;
    private static final double TOROID_SHOT_DETECT_BASELINE_ALPHA = 0.08;
    private static final boolean TOROID_IDLE_JOSTLE_ENABLED = false;
    private static final double TOROID_IDLE_JOSTLE_RPM = 15.0;
    private static final double TOROID_IDLE_JOSTLE_ON_SEC = 0.25;
    private static final double TOROID_IDLE_JOSTLE_OFF_SEC = 0.25;
    private static final double TOROID_APPROACH_START_RATIO = 0.45;
    private static final double TOROID_APPROACH_START_TICKS = TOROID_STEP_TICKS * TOROID_APPROACH_START_RATIO;
    private static final double TOROID_APPROACH_RPM = 35.0;
    private static final boolean TOROID_CCW_IS_POSITIVE = true;
    private static final boolean TOROID_TRANSIT_CCW = false;
    private static final boolean TOROID_SHOOT_CCW = true;
    private static final double TOROID_COAST_BEFORE_BRAKE_SEC = 0.15;
    private static final double TOROID_BRAKE_BEFORE_REVERSE_SEC = 0.1;
    private static final double TOROID_VELOCITY_SIGN_THRESHOLD_TPS = 40.0;
    private static final double TOROID_JAM_CURRENT_AMPS = 2.2;
    private static final double TOROID_JAM_SPEED_TPS = 40.0;
    private static final double TOROID_JAM_DETECT_SEC = 0.08;
    private static final double TOROID_STALL_DETECT_SEC = 0.12;
    private static final double TOROID_STALL_PROGRESS_TICKS = 4.0;
    private static final double TOROID_STALL_MIN_ERROR_TICKS = TOROID_POSITION_TOLERANCE_TICKS * 2.0;
    private static final double TOROID_JAM_RELAX_MAX_SEC = 0.15;
    private static final double TOROID_JAM_STOP_TPS = 10.0;
    private static final double TOROID_JAM_REVERSE_RPM = 40.0;
    private static final double TOROID_JAM_REVERSE_SEC = 0.08;
    private static final double TOROID_JAM_FORWARD_RPM = 30.0;
    private static final double TOROID_JAM_FORWARD_SEC = 0.06;
    private static final double TOROID_JAM_COOLDOWN_SEC = 0.08;
    private static final boolean TOROID_TEST_ONLY = false; // temporary: disable non-toroid actuation
    private static final double TOROID_PADDLE_SENSOR_OFFSET_DEG = 0.0;
    private static final double TOROID_PADDLE_PROX_THRESHOLD_IN = 1.1;
    private static final double TOROID_PADDLE_PROX_SOFT_IN = 0.25;
    private static final double TOROID_PADDLE_WHITE_THRESHOLD = 0.62;
    private static final double TOROID_PADDLE_WHITE_SOFT = 0.12;
    private static final double TOROID_PADDLE_ENTER_THRESHOLD = 0.55;
    private static final double TOROID_PADDLE_EXIT_THRESHOLD = 0.35;
    private static final double TOROID_BALL_PROX_THRESHOLD_IN = 1.85;
    private static final double TOROID_BALL_PROX_SOFT_IN = 0.5;
    private static final double TOROID_BALL_MATCH_SOFT = 0.2;
    private static final double TOROID_BALL_SUPPRESS_THRESHOLD = 0.35;
    private static final double TOROID_PADDLE_MIN_SEPARATION_TICKS = TOROID_TICKS_PER_REV * 0.35;
    private static final double TOROID_ZERO_BLEND = 0.25;
    private static final double TOROID_ZERO_CONFIDENCE_DECAY_SEC = 6.0;
    private static final double TOROID_ZERO_CONFIDENCE_GAIN = 0.6;
    private static final double TOROID_ZERO_SAVE_INTERVAL_SEC = 0.8;
    private static final double TOROID_ZERO_MIN_SAVE_CONFIDENCE = 0.35;
    private static final double TOROID_ZERO_MAX_TPS = 20.0;
    private static final double AUTO_TURN_DEADBAND_RATIO = 0.68; // align with precise settle ratio
    private static final double SLOT_CHECK_SETTLE_SEC = 0.25;
    private static final double SLOT_CHECK_DWELL_SEC = 0.25;
    private static final int SLOT_CHECK_BURST_SAMPLES = 5;
    private static final double SLOT_CHECK_SAMPLE_SPACING_SEC = 0.02;
    // Teleop drive scaling: higher caps = more authority; fast mode bumps to full send.
    private static final double DRIVE_TRANSLATION_CAP = 0.90;
    private static final double DRIVE_TURN_CAP = 0.90;
    private static final double AIM_ASSIST_TURN_GAIN = 2.3;
    private static final double AUTO_MIN_TRANSLATION_POWER = 0.3;
    private static final double AUTO_TRANSLATION_GAIN_SCALE = 1.15;
    private static final double AUTO_MIN_ROTATION_POWER = 0.42;
    private static final double AUTO_STRAFE_PRIORITY_DISTANCE_INCHES = 10.0;
    private static final double AUTO_STRAFE_PRIORITY_AXIS_RATIO = 1.25;
    private static final double AUTO_STRAFE_PRIORITY_TURN_SCALE = 0.35;

    // Only trust the large field tags for localization.
    private static final int[] APRIL_TAG_ALLOWED_IDS = {20, 24};
    private static final int[] GOAL_TAG_IDS = {20, 24};
    // Obelisk tags encode the green-ball position in the fixed 3-ball pattern.
    private static final int[] OBELISK_PATTERN_TAG_IDS = {21, 22, 33};
    // Obelisk faces +X on the -X perimeter; keep a tolerance so slight skew still counts.
    private static final double OBELISK_TARGET_HEADING_RADIANS = 0.0;
    private static final double OBELISK_HEADING_TOLERANCE_RADIANS = Math.toRadians(20.0);
    // Simple goal-start autonomous definitions (base on red side; blue mirrors Y/heading).
    private static final double GOAL_START_X = -53.5;
    private static final double GOAL_START_Y = 47.0;
    private static final int GOAL_RED_START_TAG_ID = 24;
    private static final int GOAL_BLUE_START_TAG_ID = 20;
    private static final double LEAVE_TARGET_X = 60;
    private static final double LEAVE_TARGET_Y = 32.0;
    private static final double LEAVE_TARGET_HEADING = Math.toRadians(180.0);
    // Audience-side start for the same leave path; heading fixed to 180 deg instead of tag-derived.
    private static final double AUDIENCE_START_X = 62.0;
    private static final double AUDIENCE_START_Y = 16.0;
    // Audience leave path: drive forward from the audience start instead of strafing.
    private static final double AUDIENCE_LEAVE_FORWARD_INCHES = 16.0;

    // Autonomous collection passes: intake faces 180 degrees (intake end forward).
    private static final double AUTO_COLLECT_HEADING_RADIANS = Math.toRadians(180.0);

    // Triad collection geometry (red side); blue mirrors across the X axis.
    private static final double TRIAD_CENTER_Y_RED = 47.25;
    private static final double TRIAD_SIDE_BALL_OFFSET_Y = 5.0; // touching 5" balls
    private static final double[] TRIAD_CENTER_XS_RED = {-12.0, 12.0, 36.0};
    private static final double TRIAD_APPROACH_Y_OFFSET = 20.0; // start farther out before first ball
    // Sweep through the far side ball so one pass can collect all 3 balls in the triad.
    private static final double TRIAD_EXIT_Y_OFFSET = TRIAD_SIDE_BALL_OFFSET_Y;
    // Non-blocking post-collect flush duration: intake keeps running while later commands can proceed.
    private static final double TRIAD_INTAKE_FLUSH_MILLIS = 7500.0;
    private static final double TRIAD_COLLECT_DRIVE_POWER_SCALE = 0.5; // slow down while driving into balls
    private static final double TRIAD_COLLECT_FORWARD_OFFSET_INCHES = 7.0; // shift robot forward so intake aligns to target
    private static final double AUDIENCE_START_HEADING = Math.toRadians(180.0);
    private static final double SHOOTING_X_DELTA_FROM_START_INCHES = -4.0;
    private static final double BACK_SHOOT_X = -16.0;
    private static final double BACK_SHOOT_Y = 12.0;
    private static final double BACK_PARK_X = 12.0;
    private static final double BACK_PARK_Y = 12.0;
    private static final double BACK_PARK_HEADING = Math.toRadians(180.0);
    private static final double BLUE_LINE_CENTER_X = -11.5;
    private static final double BLUE_LINE_CENTER_Y = -23.5;
    private static final double BLUE_LINE_X_SPACING = 24.0;
    private static final double BLUE_LINE_APPROACH_Y_OFFSET = 10.0;

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

    private static final AprilTagLibrary APRIL_TAG_LIBRARY = AprilTagGameDatabase.getDecodeTagLibrary();
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
    private final ArrayList<OpModeCommand> commandSequence = new ArrayList<>();
    private OpModeCommand currentCommand = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private boolean currentCommandActionsStarted = false;
    private OpModeState continuationState = null;
    //private final SampleMecanumDrive drive;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final OnActivatedEvaluator rb1ActivatedEvaluator;
    private final OnActivatedEvaluator lb1ActivatedEvaluator;
    private final OnActivatedEvaluator a1ActivatedEvaluator;
    private final OnActivatedEvaluator liftToggleEvaluator;
    private final OnActivatedEvaluator rb2ActivatedEvaluator;
    private final OnActivatedEvaluator lb2ActivatedEvaluator;
    private final OnActivatedEvaluator rs2ActivatedEvaluator;
    private final OnActivatedEvaluator ls2ActivatedEvaluator;
    private final OnActivatedEvaluator shooterOffEvaluator;
    private final OnActivatedEvaluator y2ActivatedEvaluator;
    private final OnActivatedEvaluator b2ActivatedEvaluator;
    private final OnActivatedEvaluator a2ActivatedEvaluator;
    private final DcMotorEx wheel;
    private final DcMotorEx intakeMotor;
    private final SampleMecanumDrive drive;
    private final Servo lift;
    private final Limelight3A limelight;
    private final GoBildaPinpointDriver pinpoint;
    // currently unused but attached
    //private final RevColorSensorV3 color1;
    //private final RevColorSensorV3 color2;
    private RevColorSensorV3 toroidSensor;
    private TouchSensor paddleLimitSwitch;
    public final VisionPortal visionPortal;
    private DcMotorEx toroidMotor;
    private double intakePowerSmoothed = 0.0;
    private IntakeJamPhase intakeJamPhase = IntakeJamPhase.NONE;
    private final ElapsedTime intakeJamTimer = new ElapsedTime();
    private final ElapsedTime intakeJamDetectTimer = new ElapsedTime();
    private final ElapsedTime intakeJamCooldownTimer = new ElapsedTime();
    private boolean intakeJamConditionActive = false;
    private double intakeJamResumePower = 0.0;
    private IntakeState intakeState = IntakeState.FORWARD;
    private long intakeObsCommandedSinceNs = Long.MIN_VALUE;
    private long intakeObsLastDetectNs = Long.MIN_VALUE;
    private boolean intakeObsCandidatePrev = false;
    private boolean intakeObsReadyForEvent = true;
    private double intakeObsCurrentBaselineA = Double.NaN;
    private double intakeObsVelocityBaselineTps = Double.NaN;
    private int intakeObsDetectedCount = 0;
    private final ElapsedTime intakeToggleHoldTimer = new ElapsedTime();
    private boolean intakeToggleHoldActive = false;
    private boolean intakeXPressedPrev = false;
    private boolean shooterEnabled = true;
    private boolean shooterReverseUnjamActive = false;
    private boolean shooterReverseUnjamVelocityGatePass = false;
    private double shooterDesiredExitVelocityIps = 0.0;
    private double shooterDesiredWheelTicksPerSecond = 0.0;
    private double shooterTransferRatio = SHOOTER_EXIT_VELOCITY_TRANSFER_BASE;
    private int shooterTransferTrimClicks = 0;
    private double autonomousIntakePower = 0.0;
    private long autonomousIntakeHoldUntilMs = 0L;
    private boolean goalTagVisible = false;
    private Pose2d recoveredPose = null;
    private int lastTagDetectionsCount = 0;
    private String lastTagDetectionIds = "";
    private Pose2d lastTagDerivedPose = null;
    private final ArrayDeque<Pose2d> tagPoseQueue = new ArrayDeque<>();
    private final ArrayDeque<TimedPose> odometryPoseHistory = new ArrayDeque<>();
    private String limelightName = "";
    private int limelightPipelineIndex = -1;
    private String limelightPipelineType = "";
    private double limelightFps = Double.NaN;
    private double limelightTempC = Double.NaN;
    private double limelightCpu = Double.NaN;
    private boolean limelightResultValid = false;
    private double limelightCaptureLatencyMs = Double.NaN;
    private double limelightTargetingLatencyMs = Double.NaN;
    private double limelightParseLatencyMs = Double.NaN;
    private double limelightStalenessMs = Double.NaN;
    private double limelightTx = Double.NaN;
    private double limelightTy = Double.NaN;
    private boolean limelightAprilTagPipelineActive = false;
    private double limelightRawPoseX = Double.NaN;
    private double limelightRawPoseY = Double.NaN;
    private double limelightRawPoseHeadingDeg = Double.NaN;
    private String limelightRawPoseSource = "none";
    private boolean limelightRejectedDefaultPose = false;
    private String limelightBotposeString = "";
    private double limelightVisionConfidence = 0.0;
    private boolean intakeEnabled = true;
    private Pose2d currentAutoDriveTarget = null;
    private boolean autoStrafePriorityActive = false;
    private ShotSolution lastShotSolution = null;
    private boolean lastShotBlockedByRim = false;
    private double lastDriveTranslationCap = DRIVE_TRANSLATION_CAP;
    private double lastDriveTurnCap = DRIVE_TURN_CAP;
    private boolean driveFrontReversed = false;
    private ObeliskPattern obeliskPattern = ObeliskPattern.UNKNOWN;
    private final int[] obeliskPatternVotes = new int[ObeliskPattern.values().length];
    private int lastObeliskTagId = -1;
    private double lastObeliskTagHeading = Double.NaN;
    private double lastObeliskTagX = Double.NaN;

    private ToroidMode toroidMode = ToroidMode.STOP;
    private TransitionPhase toroidTransitionPhase = TransitionPhase.NONE;
    private final ElapsedTime toroidTransitionTimer = new ElapsedTime();
    private ToroidJamPhase toroidJamPhase = ToroidJamPhase.NONE;
    private final ElapsedTime toroidJamTimer = new ElapsedTime();
    private final ElapsedTime toroidJamDetectTimer = new ElapsedTime();
    private final ElapsedTime toroidJamCooldownTimer = new ElapsedTime();
    private final ElapsedTime toroidStallTimer = new ElapsedTime();
    private boolean toroidJamConditionActive = false;
    private double toroidJamResumeRpm = 0.0;
    private int toroidJamResumeSign = 0;
    private int toroidTargetStepIndex = 0;
    private int toroidLastTargetSign = 0;
    private double toroidPendingTargetRpm = 0.0;
    private int toroidPendingTargetSign = 0;
    private double toroidTargetRpm = 0.0;
    private double toroidTargetTps = 0.0;
    private double toroidLastErrorAbsTicks = Double.NaN;
    private double toroidLastApproachScale = 1.0;
    private double toroidLastApproachRpm = 0.0;
    private double toroidZeroTicks = 0.0;
    private double toroidZeroConfidence = 0.0;
    private long toroidZeroSavedTimestampMs = 0L;
    private boolean toroidZeroFreshForTeleop = false;
    private boolean toroidPaddleSeen = false;
    private boolean toroidZeroInitialized = false;
    private double toroidLastPaddleObsTicks = Double.NaN;
    private double toroidLastProximityInches = Double.NaN;
    private double toroidLastWhiteBias = 0.0;
    private double toroidLastBallPresence = 0.0;
    private double toroidLastPaddlePresence = 0.0;
    private final ElapsedTime toroidZeroUpdateTimer = new ElapsedTime();
    private final ElapsedTime toroidZeroSaveTimer = new ElapsedTime();
    private boolean toroidAutoActive = false;
    private double toroidAutoTargetTicks = 0.0;
    private int toroidAutoDirectionSign = 0;
    private double toroidAutoTimeoutMillis = 0.0;
    private final ElapsedTime toroidAutoTimer = new ElapsedTime();
    private final ElapsedTime toroidAutoLoopTimer = new ElapsedTime();
    private double toroidAutoJamAccumulatedMillis = 0.0;
    private int toroidAutoExpectedShots = 0;
    private int toroidAutoDetectedShots = 0;
    private double toroidAutoWheelCurrentBaselineA = Double.NaN;
    private boolean toroidAutoShotCandidatePrev = false;
    private long toroidShotDetectCommandedSinceNs = Long.MIN_VALUE;
    private long toroidLastShotDetectTimeNs = Long.MIN_VALUE;
    private final ElapsedTime toroidIdleTimer = new ElapsedTime();
    private int toroidIdleSign = 1;
    private boolean autonomousToroidHomingEnabled = true;
    private boolean autonomousInitToroidStagingComplete = false;
    private double paddleLimitCaptureTicks = Double.NaN;
    private boolean paddleLimitSwitchSeen = false;
    private boolean paddleLimitPressedRaw = false;
    private boolean paddleLimitPressedStable = false;
    private boolean paddleLimitPressedStablePrev = false;

    private final Pose2d initialPose;
    private Pose2d autonomousStartPose = null;

    private static class TimedPose {
        long timeNs;
        Pose2d pose;
    }

    public DecodeRobotControl(AllianceColor allianceColor, Pose2d initialPose, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, boolean debugMode) {
        this.allianceColor = allianceColor;
        this.initialPose = initialPose != null ? initialPose : new Pose2d();
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.state = OpModeState.MANUAL_CONTROL;
        this.debugMode = debugMode;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(LIMELIGHT_APRILTAG_PIPELINE);
        limelight.start();
        //color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        //color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        toroidMotor = hardwareMap.get(DcMotorEx.class, TOROID_MOTOR_NAME);
        toroidMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toroidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toroidMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        try {
            toroidSensor = hardwareMap.get(RevColorSensorV3.class, TOROID_SENSOR_NAME);
        } catch (Exception e) {
            Log.w(TAG, "Toroid sensor not found: " + TOROID_SENSOR_NAME);
            toroidSensor = null;
        }
        try {
            paddleLimitSwitch = hardwareMap.get(TouchSensor.class, PADDLE_LIMIT_SWITCH_NAME);
        } catch (Exception e) {
            Log.w(TAG, "Paddle limit switch not found: " + PADDLE_LIMIT_SWITCH_NAME);
            paddleLimitSwitch = null;
        }
        paddleLimitPressedRaw = paddleLimitSwitch != null && paddleLimitSwitch.isPressed();
        paddleLimitPressedStable = paddleLimitPressedRaw;
        paddleLimitPressedStablePrev = paddleLimitPressedStable;
        toroidZeroTicks = toroidMotor.getCurrentPosition();
        toroidZeroInitialized = true;
        loadToroidZeroEstimate();
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

        visionPortal = null;

        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        a1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        liftToggleEvaluator = new OnActivatedEvaluator(() -> gamepad1.b && gamepad1.y);
        rb2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);
        lb2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_bumper);
        rs2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_stick_button);
        ls2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_stick_button);
        shooterOffEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_stick_button && gamepad2.right_stick_button);
        y2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        b2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        a2ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        lb1ActivatedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);

        drive = new SampleMecanumDrive(hardwareMap);

        lift = hardwareMap.get(Servo.class, "lift");
        lift.setPosition(1.0);

        Pose2d startPose = this.initialPose;
        configurePinpoint();

        drive.setLocalizer(new PinpointLocalizer(pinpoint, startPose));
        drive.setPoseEstimate(startPose);
        latestPoseEstimate = startPose;
        lastAprilTagFieldPosition = startPose;
    }

    private Map<String, String> logData = new ArrayMap<>();
    public Map<String, String> getLogData() {
        logData.clear();

        return logData;
    }

    public Pose2d getLatestPoseEstimate() {
        return latestPoseEstimate;
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

    static double whiteBias(double r, double g, double b) {
        double eps = 1e-6;
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double chroma = max - min;
        double whiteness = 1.0 - (chroma / (max + eps));
        double brightness = (r + g + b) / 3.0;
        double brightnessGate = smoothstep(0.04, 0.25, brightness);
        return clamp01(whiteness * brightnessGate);
    }

//    private double sampleCollectorPresence() {
//        int colorReadingMaxInt = 2 << 11;
//        double red = (double) color1.red() / colorReadingMaxInt;
//        double green = (double) color1.green() / colorReadingMaxInt;
//        double blue = (double) color1.blue() / colorReadingMaxInt;
//        double color1ProximityInches = color1.getDistance(DistanceUnit.INCH);
//        double red2 = (double) color2.red() / colorReadingMaxInt;
//        double green2 = (double) color2.green() / colorReadingMaxInt;
//        double blue2 = (double) color2.blue() / colorReadingMaxInt;
//        double color2ProximityInches = color2.getDistance(DistanceUnit.INCH);
//
//        double greenMatch = greenResonance(red, green, blue);
//        double purpleMatch = purpleResonance(red, green, blue);
//        double greenMatch2 = greenResonance(red2, green2, blue2);
//        double purpleMatch2 = purpleResonance(red2, green2, blue2);
//
//        double proxSoft = 0.5;   // inches past threshold to fade out
//        double matchSoft = 0.2;  // match past threshold to fade in
//
//        double greenPresence = colorPresence(
//                color1ProximityInches, greenMatch,
//                PRESENCE_PROXIMITY_THRESHOLD_INCHES, GREEN_MATCH_THRESHOLD,
//                proxSoft, matchSoft
//        );
//
//        double purplePresence = colorPresence(
//                color1ProximityInches, purpleMatch,
//                PRESENCE_PROXIMITY_THRESHOLD_INCHES, PURPLE_MATCH_THRESHOLD,
//                proxSoft, matchSoft
//        );
//
//        double greenPresence2 = colorPresence(
//                color2ProximityInches, greenMatch2,
//                PRESENCE_PROXIMITY_THRESHOLD_INCHES, GREEN_MATCH_THRESHOLD,
//                proxSoft, matchSoft
//        );
//
//        double purplePresence2 = colorPresence(
//                color2ProximityInches, purpleMatch2,
//                PRESENCE_PROXIMITY_THRESHOLD_INCHES, PURPLE_MATCH_THRESHOLD,
//                proxSoft, matchSoft
//        );
//
//        double collectorPresence = Math.max(
//                Math.max(greenPresence, purplePresence),
//                Math.max(greenPresence2, purplePresence2)
//        );
//
//        lastCollectorPresence = collectorPresence;
//        lastColor1ProximityInches = color1ProximityInches;
//        lastColor2ProximityInches = color2ProximityInches;
//        lastColor1GreenPresence = greenPresence;
//        lastColor1PurplePresence = purplePresence;
//        lastColor2GreenPresence = greenPresence2;
//        lastColor2PurplePresence = purplePresence2;
//
//        updateCollectorPresence(collectorPresence);
//        return collectorPresence;
//    }

    private Pose2d driveInput = new Pose2d();
    public TelemetryPacket getTelemetryPacket() {
        TelemetryPacket packet = new TelemetryPacket();

        double x = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getX();
        double y = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getY();
        double heading = latestPoseEstimate == null ? 0.0 : latestPoseEstimate.getHeading();

        packet.put("PoseX", x);
        packet.put("PoseY", y);
        packet.put("PoseHeadingDeg", Math.toDegrees(heading));
        packet.put("InitialPoseX", initialPose.getX());
        packet.put("InitialPoseY", initialPose.getY());
        packet.put("InitialPoseHeadingDeg", Math.toDegrees(initialPose.getHeading()));
        if (recoveredPose != null) {
            packet.put("RecoveredPoseX", recoveredPose.getX());
            packet.put("RecoveredPoseY", recoveredPose.getY());
            packet.put("RecoveredPoseHeadingDeg", Math.toDegrees(recoveredPose.getHeading()));
        }

        // Collector color/presence telemetry removed for toroid indexer.

        Canvas overlay = packet.fieldOverlay();
        Pose2d shooterPose = getShooterPoseEstimate();
        // Draw robot and Limelight-derived poses.
        Pose2d poseForOverlay = latestPoseEstimate;
        double sx = poseForOverlay != null ? poseForOverlay.getX() : 0.0;
        double sy = poseForOverlay != null ? poseForOverlay.getY() : 0.0;
        double sheading = poseForOverlay != null ? poseForOverlay.getHeading() : 0.0;
        double shotLen = 12;
        double shx = sx + shotLen * Math.cos(sheading);
        double shy = sy + shotLen * Math.sin(sheading);

        if (lastTagDerivedPose != null) {
            overlay.setStroke("#1E90FF");
            overlay.strokeCircle(lastTagDerivedPose.getX(), lastTagDerivedPose.getY(), 4);
            double tagHx = lastTagDerivedPose.getX() + (shotLen * Math.cos(lastTagDerivedPose.getHeading()));
            double tagHy = lastTagDerivedPose.getY() + (shotLen * Math.sin(lastTagDerivedPose.getHeading()));
            overlay.strokeLine(lastTagDerivedPose.getX(), lastTagDerivedPose.getY(), tagHx, tagHy);
            overlay.setStroke("#000000");
        }
        if (recoveredPose != null) {
            overlay.setStroke("#FF8C00");
            overlay.strokeCircle(recoveredPose.getX(), recoveredPose.getY(), 4);
            overlay.setStroke("#000000");
        }
        if (currentAutoDriveTarget != null) {
            overlay.setStroke("#FF0000");
            double tx = currentAutoDriveTarget.getX();
            double ty = currentAutoDriveTarget.getY();
            double th = currentAutoDriveTarget.getHeading();
            double tLen = 10;
            overlay.strokeCircle(tx, ty, 4)
                    .strokeLine(tx, ty, tx + (tLen * Math.cos(th)), ty + (tLen * Math.sin(th)));
            overlay.setStroke("#000000");
        }

        overlay.fillCircle(sx, sy, 5)
                .strokeLine(sx, sy, shx, shy);
        if (intakeObsLastDetectNs != Long.MIN_VALUE
                && (System.nanoTime() - intakeObsLastDetectNs) <= 500_000_000L) {
            overlay.setStroke("#1E90FF");
            overlay.strokeCircle(sx, sy, 1);
            overlay.strokeCircle(sx, sy, 2);
            overlay.setStroke("#000000");
        }
        if (toroidLastShotDetectTimeNs != Long.MIN_VALUE
                && (System.nanoTime() - toroidLastShotDetectTimeNs) <= 500_000_000L) {
            overlay.setStroke("#FF0000");
            overlay.strokeCircle(sx, sy, 2);
            overlay.strokeCircle(sx, sy, 3);
            overlay.setStroke("#000000");
        }

        // Draw raw Limelight pose last so it stays visible even when overlapping the fused robot marker.
        if (!Double.isNaN(limelightRawPoseX) && !Double.isNaN(limelightRawPoseY) && !Double.isNaN(limelightRawPoseHeadingDeg)) {
            double rawHeading = Math.toRadians(limelightRawPoseHeadingDeg);
            overlay.setStroke("#FF00FF");
            overlay.strokeCircle(limelightRawPoseX, limelightRawPoseY, 7);
            overlay.strokeCircle(limelightRawPoseX, limelightRawPoseY, 9);
            overlay.strokeLine(
                    limelightRawPoseX,
                    limelightRawPoseY,
                    limelightRawPoseX + (12.0 * Math.cos(rawHeading)),
                    limelightRawPoseY + (12.0 * Math.sin(rawHeading)));
            overlay.setStroke("#000000");
        }

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
        if (lastTagDerivedPose != null) {
            packet.put("LastTagPoseX", lastTagDerivedPose.getX());
            packet.put("LastTagPoseY", lastTagDerivedPose.getY());
            packet.put("LastTagPoseHeading", lastTagDerivedPose.getHeading());
        }

        packet.put("lastDetectionYaw", lastDetectionYaw);
        packet.put("lastDetectionBearing", lastDetectionBearing);
        packet.put("lastDetectionElevation", lastDetectionElevation);
        packet.put("lastDetectionRange", lastDetectionRange);
        packet.put("AprilTagDetections", lastTagDetectionsCount);
        packet.put("AprilTagDetectionIds", lastTagDetectionIds);
        packet.put("LLName", limelightName);
        packet.put("LLPipelineIndex", limelightPipelineIndex);
        packet.put("LLPipelineType", limelightPipelineType);
        packet.put("LLFps", limelightFps);
        packet.put("LLTempC", limelightTempC);
        packet.put("LLCpuPct", limelightCpu);
        packet.put("LLValid", limelightResultValid);
        packet.put("LLCaptureLatencyMs", limelightCaptureLatencyMs);
        packet.put("LLTargetingLatencyMs", limelightTargetingLatencyMs);
        packet.put("LLParseLatencyMs", limelightParseLatencyMs);
        packet.put("LLStalenessMs", limelightStalenessMs);
        packet.put("LLTotalLatencyMs", limelightCaptureLatencyMs + limelightTargetingLatencyMs + limelightParseLatencyMs + limelightStalenessMs);
        packet.put("LLTx", limelightTx);
        packet.put("LLTy", limelightTy);
        packet.put("LLAprilTagPipeline", limelightAprilTagPipelineActive);
        packet.put("LLRawPoseX", limelightRawPoseX);
        packet.put("LLRawPoseY", limelightRawPoseY);
        packet.put("LLRawPoseHeadingDeg", limelightRawPoseHeadingDeg);
        packet.put("LLRawPoseSource", limelightRawPoseSource);
        packet.put("LLRejectedDefaultPose", limelightRejectedDefaultPose);
        packet.put("LLBotposeString", limelightBotposeString);
        packet.put("LLVisionConfidence", limelightVisionConfidence);

        packet.put("G2_RSX", gamepad2.right_stick_x);
        packet.put("WheelCurrent", wheel.getCurrent(CurrentUnit.MILLIAMPS));
        double wheelVelocityTps = wheel.getVelocity();
        packet.put("WheelVelocity", wheelVelocityTps);
        packet.put("WheelVelocityInchesPerSecond", (wheelVelocityTps / WHEEL_PPR) * SHOOTER_WHEEL_CIRCUMFERENCE_INCHES);
        packet.put("WheelVelocityError", shooterDesiredWheelTicksPerSecond - wheelVelocityTps);
        packet.put("ShooterEnabled", shooterEnabled);
        packet.put("ShooterReverseUnjamActive", shooterReverseUnjamActive);
        packet.put("ShooterReverseUnjamVelocityGatePass", shooterReverseUnjamVelocityGatePass);
        packet.put("ShooterReverseUnjamMaxWheelTps", SHOOTER_REVERSE_UNJAM_MAX_WHEEL_TPS);
        packet.put("ShooterReverseUnjamGesture",
                gamepad2.left_trigger >= SHOOTER_REVERSE_UNJAM_TRIGGER_THRESHOLD
                        && gamepad2.right_trigger >= SHOOTER_REVERSE_UNJAM_TRIGGER_THRESHOLD);
        packet.put("ShooterDesiredExitVelocityIps", shooterDesiredExitVelocityIps);
        packet.put("WheelDesiredRevPerSecond", shooterDesiredWheelTicksPerSecond / WHEEL_PPR);
        packet.put("WheelDesiredTickPerSecond", shooterDesiredWheelTicksPerSecond);
        packet.put("WheelEncoder", wheel.getCurrentPosition());
        double intakeVelocityTicksPerSec = intakeMotor.getVelocity();
        double intakeVelocityInchesPerSec = intakeVelocityTicksPerSec / INTAKE_TICKS_PER_INCH;
        packet.put("IntakeCurrent", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("IntakePower", intakeMotor.getPower());
        packet.put("IntakeState", intakeState.name());
        packet.put("IntakeEnabled", intakeEnabled);
        packet.put("IntakeToggleHoldSec", intakeToggleHoldActive ? intakeToggleHoldTimer.seconds() : 0.0);
        packet.put("IntakeToggleHoldTargetSec", INTAKE_TOGGLE_HOLD_SEC);
        packet.put("IntakeVelocityTicksPerSec", intakeVelocityTicksPerSec);
        packet.put("IntakeVelocityInchesPerSec", intakeVelocityInchesPerSec);
        packet.put("IntakeObsCount", intakeObsDetectedCount);
        packet.put("IntakeObsCurrentBaselineA", intakeObsCurrentBaselineA);
        packet.put("IntakeObsVelocityBaselineTps", intakeObsVelocityBaselineTps);
        packet.put("IntakeObsReadyForEvent", intakeObsReadyForEvent);
        packet.put("IntakeObsArmedSec", intakeObsCommandedSinceNs == Long.MIN_VALUE
                ? 0.0
                : (System.nanoTime() - intakeObsCommandedSinceNs) / 1_000_000_000.0);
        packet.put("AutonIntakeHoldRemainingSec",
                autonomousIntakeHoldUntilMs > 0L
                        ? Math.max(0.0, (autonomousIntakeHoldUntilMs - System.currentTimeMillis()) / 1000.0)
                        : 0.0);
        packet.put("IntakeJamPhase", intakeJamPhase.name());
        packet.put("IntakeJamCondition", intakeJamConditionActive);
        packet.put("IntakeJamDetectSec", intakeJamDetectTimer.seconds());
        packet.put("IntakeJamTimerSec", intakeJamTimer.seconds());
        packet.put("IntakeJamResumePower", intakeJamResumePower);
        packet.put("IntakeJamCooldownSec", intakeJamCooldownTimer.seconds());
        packet.put("DriveInputX", driveInput.getX());
        packet.put("DriveInputY", driveInput.getY());
        packet.put("DriveFrontReversed", driveFrontReversed);
        packet.put("DriveTranslationCap", lastDriveTranslationCap);
        packet.put("DriveTurnCap", lastDriveTurnCap);
        if (currentAutoDriveTarget != null) {
            Pose2d autoError = getPoseTargetError(currentAutoDriveTarget);
            Pose2d autoCommand = getPoseTargetAutoDriveControl(currentAutoDriveTarget);
            if (autoError != null) {
                packet.put("AutoErrFieldX", autoError.getX());
                packet.put("AutoErrFieldY", autoError.getY());
                packet.put("AutoErrHeadingDeg", Math.toDegrees(autoError.getHeading()));
                packet.put("AutoErrDistance", Math.hypot(autoError.getX(), autoError.getY()));
            }
            packet.put("AutoCmdRobotX", autoCommand.getX());
            packet.put("AutoCmdRobotY", autoCommand.getY());
            packet.put("AutoCmdTurn", autoCommand.getHeading());
            packet.put("AutoStrafePriority", autoStrafePriorityActive);
        }
        Double[] driveMotorPowers = drive.getMotorPowers();
        Double[] driveMotorVelocities = drive.getMotorVelocities();
        packet.put("DrivePwrFL", driveMotorPowers[0]);
        packet.put("DrivePwrBL", driveMotorPowers[1]);
        packet.put("DrivePwrBR", driveMotorPowers[2]);
        packet.put("DrivePwrFR", driveMotorPowers[3]);
        packet.put("DriveVelFL", driveMotorVelocities[0]);
        packet.put("DriveVelBL", driveMotorVelocities[1]);
        packet.put("DriveVelBR", driveMotorVelocities[2]);
        packet.put("DriveVelFR", driveMotorVelocities[3]);
        packet.put("ToroidMode", toroidMode.name());
        packet.put("ToroidTargetRpm", toroidTargetRpm);
        packet.put("ToroidTargetTps", toroidTargetTps);
        packet.put("ToroidActualTps", toroidMotor != null ? toroidMotor.getVelocity() : 0.0);
        packet.put("ToroidCurrentA", toroidMotor != null ? toroidMotor.getCurrent(CurrentUnit.AMPS) : 0.0);
        packet.put("ToroidPosTicks", toroidMotor != null ? toroidMotor.getCurrentPosition() : 0);
        packet.put("ToroidTargetStep", toroidTargetStepIndex);
        double toroidTargetPosTicks = toroidAutoActive
                ? toroidAutoTargetTicks
                : (toroidZeroTicks + toroidTargetStepIndex * TOROID_STEP_TICKS);
        packet.put("ToroidTargetPosTicks", (int) Math.round(toroidTargetPosTicks));
        packet.put("ToroidTransition", toroidTransitionPhase.name());
        packet.put("ToroidJamPhase", toroidJamPhase.name());
        packet.put("ToroidJamCondition", toroidJamConditionActive);
        packet.put("ToroidJamDetectSec", toroidJamDetectTimer.seconds());
        packet.put("ToroidJamTimerSec", toroidJamTimer.seconds());
        packet.put("ToroidJamResumeRpm", toroidJamResumeRpm);
        packet.put("ToroidJamCooldownSec", toroidJamCooldownTimer.seconds());
        packet.put("ToroidAutoElapsedMs", toroidAutoTimer.milliseconds());
        packet.put("ToroidAutoJamAccumulatedMs", toroidAutoJamAccumulatedMillis);
        packet.put("ToroidAutoEffectiveElapsedMs", Math.max(0.0, toroidAutoTimer.milliseconds() - toroidAutoJamAccumulatedMillis));
        packet.put("ToroidAutoExpectedShots", toroidAutoExpectedShots);
        packet.put("ToroidAutoDetectedShots", toroidAutoDetectedShots);
        packet.put("ToroidAutoShotCurrentBaselineA", toroidAutoWheelCurrentBaselineA);
        packet.put("ToroidApproachScale", toroidLastApproachScale);
        packet.put("ToroidApproachRpm", toroidLastApproachRpm);
        packet.put("ToroidStallSec", toroidStallTimer.seconds());
        packet.put("ToroidErrorAbsTicks", toroidLastErrorAbsTicks);
        packet.put("ToroidZeroTicks", toroidZeroTicks);
        packet.put("ToroidZeroDeg", toroidZeroTicks * 360.0 / TOROID_TICKS_PER_REV);
        packet.put("ToroidZeroConf", toroidZeroConfidence);
        packet.put("ToroidZeroSavedAgeSec", toroidZeroSavedTimestampMs > 0
                ? (System.currentTimeMillis() - toroidZeroSavedTimestampMs) / 1000.0
                : Double.NaN);
        packet.put("ToroidZeroFreshForTeleop", toroidZeroFreshForTeleop);
        packet.put("ToroidPaddleSeen", toroidPaddleSeen);
        packet.put("ToroidPaddlePresence", toroidLastPaddlePresence);
        packet.put("ToroidBallPresence", toroidLastBallPresence);
        packet.put("ToroidWhiteBias", toroidLastWhiteBias);
        packet.put("ToroidProxIn", toroidLastProximityInches);
        packet.put("PaddleLimitPressed", paddleLimitPressedStable);
        packet.put("PaddleLimitRaw", paddleLimitPressedRaw);
        packet.put("PaddleLimitPresent", paddleLimitSwitch != null);
        packet.put("PaddleLimitSeen", paddleLimitSwitchSeen);
        packet.put("PaddleLimitCaptureTicks", paddleLimitCaptureTicks);
        packet.put("AutonInitToroidStaged", autonomousInitToroidStagingComplete);
        packet.put("PaddleHomeOffsetDeg", TOROID_HOME_OFFSET_DEGREES);
        packet.put("PaddleAutonStartOffsetDeg", TOROID_AUTON_START_OFFSET_DEGREES);
        packet.put("ObeliskPattern", obeliskPattern.name());
        packet.put("ObeliskTagId", lastObeliskTagId);
        packet.put("ObeliskTagHeading", lastObeliskTagHeading);
        packet.put("ObeliskTagX", lastObeliskTagX);
        packet.put("ObeliskVotesGreenFirst", obeliskPatternVotes[ObeliskPattern.GREEN_FIRST.ordinal()]);
        packet.put("ObeliskVotesGreenMiddle", obeliskPatternVotes[ObeliskPattern.GREEN_MIDDLE.ordinal()]);
        packet.put("ObeliskVotesGreenLast", obeliskPatternVotes[ObeliskPattern.GREEN_LAST.ordinal()]);
        // Shot solution telemetry removed for clarity.
        packet.put("ShooterTransferRatio", shooterTransferRatio);
        packet.put("ShooterTransferTrimClicks", shooterTransferTrimClicks);
        packet.put("ShootAimError", getShooterHeadingError());

        packet.put("PinpointHeading", pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS));
        packet.put("PinpointX", pinpoint.getEncoderX());
        packet.put("PinpointY", pinpoint.getEncoderY());

        return packet;
    }

    public void autonomousInit(AutonomousPlan autonomousPlan, Pose2d startPose) {
        timeSinceInit.reset();
        isAutonomous = true;
        autonomousToroidHomingEnabled = false;
        autonomousInitToroidStagingComplete = false;
        paddleLimitCaptureTicks = Double.NaN;
        shooterTransferRatio = SHOOTER_EXIT_VELOCITY_TRANSFER_BASE;
        assumeToroidHomeAtStartupForAutonomous();
        Pose2d poseToUse = startPose != null ? startPose : getStartPoseForPlan(allianceColor, autonomousPlan);
        drive.setPoseEstimate(poseToUse);
        lastAprilTagFieldPosition = poseToUse;
        latestPoseEstimate = poseToUse;
        odometryPoseHistory.clear();
        recordOdometryPose(poseToUse);
        autonomousStartPose = poseToUse;
        recoveredPose = null;
        setCommandSequence(buildAutonomousCommands(autonomousPlan));
    }

    private void assumeToroidHomeAtStartupForAutonomous() {
        if (toroidMotor == null) {
            return;
        }
        int currentPosition = toroidMotor.getCurrentPosition();
        double offsetTicks = TOROID_TICKS_PER_REV * (TOROID_HOME_OFFSET_DEGREES / 360.0);
        // Autonomous startup assumption: paddle begins at home.
        // Keep limit-seen false so first real switch hit can still re-anchor zero.
        toroidZeroTicks = normalizeTicks(currentPosition - offsetTicks, TOROID_TICKS_PER_REV);
        toroidZeroConfidence = 1.0;
        toroidZeroInitialized = true;
        paddleLimitSwitchSeen = false;
    }

    public void teleopInit(Pose2d startPose) {
        timeSinceInit.reset();
        autonomousToroidHomingEnabled = true;
        autonomousInitToroidStagingComplete = false;
        toroidZeroFreshForTeleop = isToroidZeroFreshNow();
        // Only trust persisted zero/home if it was written recently.
        paddleLimitSwitchSeen = toroidZeroFreshForTeleop;
        Pose2d poseToUse = startPose != null ? startPose : this.initialPose;
        drive.setPoseEstimate(poseToUse);
        lastAprilTagFieldPosition = poseToUse;
        latestPoseEstimate = poseToUse;
        odometryPoseHistory.clear();
        recordOdometryPose(poseToUse);
        driveFrontReversed = false;
        recoveredPose = poseToUse;
    }

    public void forcePoseEstimate(Pose2d pose) {
        if (pose == null) {
            return;
        }
        drive.setPoseEstimate(pose);
        lastAprilTagFieldPosition = pose;
        latestPoseEstimate = pose;
        recordOdometryPose(pose);
        recoveredPose = pose;
    }

    public void refreshPoseEstimate() {
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        recordOdometryPose(latestPoseEstimate);
    }

    public void initializeMechanicalBlocking() {
        state = OpModeState.MANUAL_CONTROL;
    }

    public void updateAutonomousInitToroidStaging() {
        if (!isAutonomous || toroidMotor == null) {
            return;
        }
        if (paddleLimitSwitch == null) {
            toroidMotor.setPower(0.0);
            autonomousInitToroidStagingComplete = true;
            return;
        }

        if (toroidMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            toroidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int currentPosition = toroidMotor.getCurrentPosition();
        updatePaddleLimitSwitchState();
        if (paddleLimitPressedStable && !paddleLimitPressedStablePrev) {
            captureToroidHomeFromLimitSwitch(currentPosition);
        }
        paddleLimitPressedStablePrev = paddleLimitPressedStable;

        if (!paddleLimitSwitchSeen) {
            int seekSign = getToroidDirectionSign(TOROID_TRANSIT_CCW);
            toroidMotor.setVelocity(rpmToTicksPerSec(TOROID_AUTON_INIT_SEEK_RPM * seekSign, TOROID_TICKS_PER_REV));
            autonomousInitToroidStagingComplete = false;
            return;
        }

        if (Double.isNaN(paddleLimitCaptureTicks)) {
            paddleLimitCaptureTicks = currentPosition;
        }
        double startOffsetTicks = TOROID_TICKS_PER_REV * (TOROID_AUTON_START_OFFSET_DEGREES / 360.0);
        double startTargetTicks = paddleLimitCaptureTicks + startOffsetTicks;
        double errorTicks = startTargetTicks - currentPosition;
        if (Math.abs(errorTicks) <= TOROID_POSITION_TOLERANCE_TICKS) {
            toroidMotor.setPower(0.0);
            autonomousInitToroidStagingComplete = true;
            return;
        }

        double directionSign = Math.signum(errorTicks);
        toroidMotor.setVelocity(rpmToTicksPerSec(TOROID_AUTON_INIT_STAGE_RPM * directionSign, TOROID_TICKS_PER_REV));
        autonomousInitToroidStagingComplete = false;
    }

    public void startup(OpModeState startupState) {
        timeSinceStart.reset();
        timeInState.reset();
        state = startupState;
    }

    private void evaluateSwitchCamera() {
        // One Limelight camera only; pipeline is selected during init.
    }

    public boolean evaluate() {
        double dt = loopTime.milliseconds();
        loopTime.reset();
        if (TOROID_TEST_ONLY) {
            updateToroidControl();
            setDrivePower(new Pose2d());
            if (rs2ActivatedEvaluator.evaluate()) {
                shooterEnabled = !shooterEnabled;
            }
            if (ls2ActivatedEvaluator.evaluate()) {
                shooterEnabled = false;
            }
            if (shooterOffEvaluator.evaluate()) {
                shooterEnabled = !shooterEnabled;
            }
            updateShooterControl(true);
            updateIntakeToggleFromX();
            boolean intakeStopHeld = gamepad2.x;
            boolean manualIntakeReverse = gamepad2.dpad_down;
            if (intakeStopHeld) {
                intakeState = IntakeState.OFF;
            } else if (manualIntakeReverse) {
                intakeState = IntakeState.REVERSE_REJECT;
            } else if (intakeEnabled) {
                intakeState = IntakeState.FORWARD;
            } else {
                intakeState = IntakeState.OFF;
            }
            if (toroidJamPhase != ToroidJamPhase.NONE) {
                intakeState = IntakeState.OFF;
            }
            double intakePowerTarget = 0.0;
            if (intakeState == IntakeState.FORWARD) {
                intakePowerTarget = INTAKE_MOTOR_POWER;
            } else if (intakeState == IntakeState.REVERSE_REJECT) {
                intakePowerTarget = -INTAKE_MOTOR_POWER;
            }
            updateIntakePower(intakePowerTarget, dt);
            lift.setPosition(1.0);
            updateIntakeObservationDetection();
            updateToroidShotDetection();
            return state != OpModeState.HALT_OPMODE;
        }
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();
        recordOdometryPose(latestPoseEstimate);
        evaluateSwitchCamera();
        evaluatePositioningSystems();

        OpModeState currentState = state;
        OpModeState nextState = currentState;
        switch (currentState) {
            case MANUAL_CONTROL:
                nextState = evaluateManualControl(dt);
                break;
            case COMMAND_SEQUENCE:
                nextState = evaluateCommandSequence(dt);
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
        updateIntakeObservationDetection();
        updateToroidShotDetection();

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
        return allianceColor == AllianceColor.RED
                ? RED_BASKET_POSITION_INCHES
                : mirrorVectorForBlue(RED_BASKET_POSITION_INCHES);
    }

    private static Vector2d mirrorVectorForBlue(Vector2d vector) {
        return new Vector2d(vector.getX(), -vector.getY());
    }

    private Pose2d getShooterPoseEstimate() {
        Pose2d basePose = latestPoseEstimate != null ? latestPoseEstimate : lastAprilTagFieldPosition;
        if (basePose == null) return null;
        double heading = basePose.getHeading();
        return getShooterPoseFromRobotPose(basePose, heading);
    }

    private Pose2d getShooterPoseFromRobotPose(Pose2d robotPose, double heading) {
        double offsetX = (SHOOTER_FORWARD_OFFSET_INCHES * Math.cos(heading)) -
                (SHOOTER_LATERAL_OFFSET_INCHES * Math.sin(heading));
        double offsetY = (SHOOTER_FORWARD_OFFSET_INCHES * Math.sin(heading)) +
                (SHOOTER_LATERAL_OFFSET_INCHES * Math.cos(heading));
        return new Pose2d(
                robotPose.getX() + offsetX,
                robotPose.getY() + offsetY,
                heading);
    }

    private double getAimHeadingFromRobotPose(Pose2d robotPose, Vector2d basket) {
        if (robotPose == null || basket == null) {
            return 0.0;
        }
        double heading = robotPose.getHeading();
        if (Double.isNaN(heading)) {
            heading = Math.atan2(basket.getY() - robotPose.getY(), basket.getX() - robotPose.getX());
        }
        for (int i = 0; i < 3; i++) {
            Pose2d shooterPose = getShooterPoseFromRobotPose(robotPose, heading);
            heading = Math.atan2(
                    basket.getY() - shooterPose.getY(),
                    basket.getX() - shooterPose.getX());
        }
        return Angle.norm(heading);
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
        double backSpinRadPerSec = 0.0;

        for (int i = 0; i < 3; i++) {
            double denom = 2.0 * cosTheta * cosTheta * verticalTerm;
            exitVelocityIps = Math.sqrt((effectiveGravity * horizontalDistance * horizontalDistance) / denom);
            double tangentialSpeedIps = exitVelocityIps / transferRatio;
            double naturalRotations = SHOOTER_CONTACT_ARC_LENGTH_INCHES / (2 * Math.PI * BALL_RADIUS_INCHES);
            double avgLinear = Math.max(1e-3, 0.5 * (tangentialSpeedIps + exitVelocityIps));
            double contactTime = SHOOTER_CONTACT_ARC_LENGTH_INCHES / avgLinear;
            double rollSpinRadPerSec = (naturalRotations * 2 * Math.PI) / Math.max(1e-3, contactTime);
            backSpinRadPerSec = (exitVelocityIps / BALL_RADIUS_INCHES)
                    * SHOOTER_SPIN_GEOMETRY_RATIO
                    * SHOOTER_SPIN_EFFICIENCY;
            double liftRatio = Math.max(0.0, BACKSPIN_LIFT_PER_RAD_PER_SEC * backSpinRadPerSec);
            double magnusMultiplier = Math.max(0.1, 1.0 - liftRatio);
            effectiveGravity = BALLISTIC_GRAVITY_IN_PER_S2 * magnusMultiplier;
        }

        double closeBoost = horizontalDistance <= SHOOTER_CLOSE_RANGE_MAX_DISTANCE_INCHES
                ? (1.0 + SHOOTER_CLOSE_RANGE_BOOST)
                : 1.0;
        exitVelocityIps *= closeBoost;
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
        solution.backSpinRadPerSec = backSpinRadPerSec;
        solution.transferRatio = transferRatio;
        solution.tangentialSpeedIps = exitVelocityIps / transferRatio;
        return solution;
    }

    private void updateShooterControl(boolean allowGamepadTrim) {
        double wheelTpsAbs = wheel != null ? Math.abs(wheel.getVelocity()) : 0.0;
        shooterReverseUnjamVelocityGatePass = wheelTpsAbs <= SHOOTER_REVERSE_UNJAM_MAX_WHEEL_TPS;
        shooterReverseUnjamActive = isShooterReverseUnjamGestureActive() && shooterReverseUnjamVelocityGatePass;
        if (shooterReverseUnjamActive) {
            shooterDesiredExitVelocityIps = 0.0;
            shooterDesiredWheelTicksPerSecond = -rpmToTicksPerSec(SHOOTER_REVERSE_UNJAM_RPM, WHEEL_PPR);
            wheel.setVelocity(shooterDesiredWheelTicksPerSecond);
            lastShotBlockedByRim = false;
            lastShotSolution = null;
            return;
        }
        ShotSolution shotSolution = null;
        lastShotBlockedByRim = false;
        if (allowGamepadTrim) {
            // Bumper-based trim clicks only.
        }
        double clickTrim = shooterTransferTrimClicks * SHOOTER_TRANSFER_CLICK_STEP;
        double transferRatioBase = SHOOTER_EXIT_VELOCITY_TRANSFER_BASE + clickTrim;
        shooterTransferRatio = Range.clip(
                transferRatioBase,
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
    }

    private void updateIntakePower(double targetPower, double dtMillis) {
        double maxDelta = INTAKE_POWER_SLEW_PER_SEC * (dtMillis / 1000.0);
        double delta = Range.clip(targetPower - intakePowerSmoothed, -maxDelta, maxDelta);
        intakePowerSmoothed = Range.clip(intakePowerSmoothed + delta, -1.0, 1.0);
        double actualTps = intakeMotor.getVelocity();
        double currentAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
        double requestedSign = Math.abs(intakePowerSmoothed) < 1e-3 ? 0.0 : Math.signum(intakePowerSmoothed);
        boolean jamAllowed = requestedSign != 0.0
                && intakeJamPhase == IntakeJamPhase.NONE
                && intakeJamCooldownTimer.seconds() >= INTAKE_JAM_COOLDOWN_SEC;
        boolean jamCondition = jamAllowed
                && Math.abs(actualTps) <= INTAKE_JAM_SPEED_TPS
                && currentAmps >= INTAKE_JAM_CURRENT_AMPS;
        if (jamCondition) {
            if (!intakeJamConditionActive) {
                intakeJamConditionActive = true;
                intakeJamDetectTimer.reset();
            }
            if (intakeJamDetectTimer.seconds() >= INTAKE_JAM_DETECT_SEC) {
                intakeJamPhase = IntakeJamPhase.RELAX;
                intakeJamTimer.reset();
                intakeJamResumePower = intakePowerSmoothed;
                intakeJamConditionActive = false;
                intakeJamDetectTimer.reset();
            }
        } else {
            intakeJamConditionActive = false;
        }

        if (intakeJamPhase != IntakeJamPhase.NONE) {
            if (intakeJamPhase == IntakeJamPhase.RELAX) {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intakeMotor.setPower(0.0);
                if (Math.abs(actualTps) <= INTAKE_JAM_STOP_TPS
                        || intakeJamTimer.seconds() >= INTAKE_JAM_RELAX_MAX_SEC) {
                    intakeJamPhase = IntakeJamPhase.REVERSE;
                    intakeJamTimer.reset();
                }
            } else if (intakeJamPhase == IntakeJamPhase.REVERSE) {
                if (Math.abs(intakeJamResumePower) < 1e-3) {
                    intakeJamPhase = IntakeJamPhase.NONE;
                    intakeJamCooldownTimer.reset();
                } else {
                    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    double reversePower = -Math.signum(intakeJamResumePower) * INTAKE_JAM_REVERSE_POWER;
                    intakeMotor.setPower(reversePower);
                    if (intakeJamTimer.seconds() >= INTAKE_JAM_REVERSE_SEC) {
                        intakeJamPhase = IntakeJamPhase.NONE;
                        intakeJamCooldownTimer.reset();
                    }
                }
            }
            return;
        }

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(intakePowerSmoothed);
    }

    private void updateIntakeObservationDetection() {
        if (intakeMotor == null) {
            intakeObsCandidatePrev = false;
            intakeObsCommandedSinceNs = Long.MIN_VALUE;
            return;
        }
        if (intakeJamPhase != IntakeJamPhase.NONE) {
            intakeObsCandidatePrev = false;
            intakeObsReadyForEvent = true;
            return;
        }

        long nowNs = System.nanoTime();
        boolean intakeCommanded = intakePowerSmoothed > 0.15;
        if (!intakeCommanded) {
            intakeObsCommandedSinceNs = Long.MIN_VALUE;
            intakeObsCandidatePrev = false;
            intakeObsReadyForEvent = true;
            return;
        }
        if (intakeObsCommandedSinceNs == Long.MIN_VALUE) {
            intakeObsCommandedSinceNs = nowNs;
        }
        if ((nowNs - intakeObsCommandedSinceNs) < (long) (INTAKE_OBS_ARM_MILLIS * 1_000_000.0)) {
            intakeObsCandidatePrev = false;
            return;
        }

        double currentA = intakeMotor.getCurrent(CurrentUnit.AMPS);
        double actualTps = Math.abs(intakeMotor.getVelocity());
        if (Double.isNaN(currentA) || Double.isNaN(actualTps)) {
            intakeObsCandidatePrev = false;
            return;
        }
        if (Double.isNaN(intakeObsCurrentBaselineA)) {
            intakeObsCurrentBaselineA = currentA;
        }
        if (Double.isNaN(intakeObsVelocityBaselineTps)) {
            intakeObsVelocityBaselineTps = actualTps;
        }

        boolean velocityDroop = actualTps <= intakeObsVelocityBaselineTps * (1.0 - INTAKE_OBS_VELOCITY_DROP_RATIO);
        double currentDeltaA = currentA - intakeObsCurrentBaselineA;
        boolean currentSpike = currentDeltaA >= INTAKE_OBS_CURRENT_SPIKE_AMPS;
        boolean candidate = velocityDroop && currentSpike;
        boolean risingEdge = candidate && !intakeObsCandidatePrev && intakeObsReadyForEvent;
        intakeObsCandidatePrev = candidate;

        double baselineAlpha = candidate ? INTAKE_OBS_BASELINE_ALPHA_WHILE_CANDIDATE : INTAKE_OBS_BASELINE_ALPHA;
        intakeObsCurrentBaselineA += (currentA - intakeObsCurrentBaselineA) * baselineAlpha;
        intakeObsVelocityBaselineTps += (actualTps - intakeObsVelocityBaselineTps) * baselineAlpha;

        boolean currentRecovered = currentA <= (intakeObsCurrentBaselineA + INTAKE_OBS_REARM_CURRENT_MARGIN_AMPS);
        boolean velocityRecovered = actualTps >= (intakeObsVelocityBaselineTps * (1.0 - INTAKE_OBS_REARM_VELOCITY_MARGIN_RATIO));
        if (currentRecovered && velocityRecovered) {
            intakeObsReadyForEvent = true;
        }

        if (!risingEdge) {
            return;
        }
        boolean refractoryElapsed = intakeObsLastDetectNs == Long.MIN_VALUE
                || (nowNs - intakeObsLastDetectNs) >= (long) (INTAKE_OBS_REFRACTORY_MILLIS * 1_000_000.0);
        if (!refractoryElapsed) {
            return;
        }

        intakeObsDetectedCount++;
        intakeObsLastDetectNs = nowNs;
        intakeObsReadyForEvent = false;
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

    private boolean isShooterReverseUnjamGestureActive() {
        return gamepad2.left_trigger >= SHOOTER_REVERSE_UNJAM_TRIGGER_THRESHOLD
                && gamepad2.right_trigger >= SHOOTER_REVERSE_UNJAM_TRIGGER_THRESHOLD;
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
        double backSpinRadPerSec;
        double transferRatio;
        double tangentialSpeedIps;
    }

    private OpModeState evaluateManualControl(double dtMillis) {
        // Default shooter on; combo press toggles it.
        if (shooterOffEvaluator.evaluate()) {
            shooterEnabled = !shooterEnabled;
        }

        boolean trimFasterRequest = rb2ActivatedEvaluator.evaluate();
        boolean trimSlowerRequest = lb2ActivatedEvaluator.evaluate();

        //sampleCollectorPresence(); // keep telemetry updated; no longer drives intake control
        if (trimFasterRequest && shooterTransferTrimClicks > -SHOOTER_TRANSFER_CLICK_LIMIT) {
            shooterTransferTrimClicks--;
        } else if (trimSlowerRequest && shooterTransferTrimClicks < SHOOTER_TRANSFER_CLICK_LIMIT) {
            shooterTransferTrimClicks++;
        }

        updateShooterControl(true);

        updateToroidControl();

        if (liftToggleEvaluator.evaluate()) {
            lifted = !lifted;
            if (lifted) {
                shooterEnabled = false;
                intakeEnabled = false;
            }
        }

        lift.setPosition(lifted ? 0.0 : 1.0);

        updateIntakeToggleFromX();
        boolean intakeStopHeld = gamepad2.x;
        boolean manualIntakeReverse = gamepad2.dpad_down;

        // Manual-only intake control: dpad down reverses; hold X to stop.
        if (intakeStopHeld) {
            intakeState = IntakeState.OFF;
        } else if (manualIntakeReverse) {
            intakeState = IntakeState.REVERSE_REJECT;
        } else if (intakeEnabled) {
            intakeState = IntakeState.FORWARD;
        } else {
            intakeState = IntakeState.OFF;
        }
        if (toroidJamPhase != ToroidJamPhase.NONE) {
            intakeState = IntakeState.OFF;
        }

        double intakePowerTarget = 0.0;
        if (intakeState == IntakeState.FORWARD) {
            intakePowerTarget = INTAKE_MOTOR_POWER;
        } else if (intakeState == IntakeState.REVERSE_REJECT) {
            intakePowerTarget = -INTAKE_MOTOR_POWER;
        } else if (intakeState == IntakeState.OFF) {
            intakePowerTarget = 0.0;
        }
        updateIntakePower(intakePowerTarget, dtMillis);

        if (rb1ActivatedEvaluator.evaluate()) {
            driveFrontReversed = !driveFrontReversed;
        }

        driveInput = getRobotRelativeDriveInput(gamepad1, driveFrontReversed);
        double translationCap = DRIVE_TRANSLATION_CAP;
        double turnCap = DRIVE_TURN_CAP;
        lastDriveTranslationCap = translationCap;
        lastDriveTurnCap = turnCap;
        driveInput = capDriveInput(driveInput, translationCap, turnCap);

        Pose2d driveCommand = driveInput;
        boolean aimAssistActive = gamepad1.right_stick_button && latestPoseEstimate != null;
        if (aimAssistActive) {
            Vector2d basket = getActiveBasketPosition();
            double desiredHeading = getAimHeadingFromRobotPose(latestPoseEstimate, basket);
            double headingError = Angle.normDelta(desiredHeading - latestPoseEstimate.getHeading());
            double turnCommand = Range.clip(headingError * AIM_ASSIST_TURN_GAIN, -turnCap, turnCap);
            driveCommand = new Pose2d(driveInput.getX(), driveInput.getY(), turnCommand);
        }

        setDrivePower(driveCommand);
        return OpModeState.MANUAL_CONTROL;
    }

    private void updateIntakeToggleFromX() {
        boolean xPressed = gamepad2.x;
        boolean xRisingEdge = xPressed && !intakeXPressedPrev;

        if (intakeEnabled) {
            if (xPressed) {
                if (!intakeToggleHoldActive) {
                    intakeToggleHoldActive = true;
                    intakeToggleHoldTimer.reset();
                } else if (intakeToggleHoldTimer.seconds() >= INTAKE_TOGGLE_HOLD_SEC) {
                    intakeEnabled = false;
                    intakeToggleHoldActive = false;
                }
            } else {
                intakeToggleHoldActive = false;
            }
        } else {
            // Once disabled, a normal tap re-enables intake (no long hold required).
            if (xRisingEdge) {
                intakeEnabled = true;
            }
            intakeToggleHoldActive = false;
        }

        intakeXPressedPrev = xPressed;
    }

    private Pose2d getRobotRelativeDriveInput(Gamepad gamepad, boolean frontReversed) {
        final double reverseMultiplier = frontReversed ? -1.0 : 1.0;

        // Robot-frame cardinal sanity check: D-pad drives pure cardinal.
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
            return new Pose2d(robotX * reverseMultiplier, robotY * reverseMultiplier, 0.0);
        }

        // Left stick = translation in robot frame.
        double robotX = applySignedSquareDeadband(-gamepad.left_stick_y, 0.02);
        double robotY = applySignedSquareDeadband(-gamepad.left_stick_x, 0.02);
        double mag = Math.hypot(robotX, robotY);
        if (mag > 1.0) {
            robotX /= mag;
            robotY /= mag;
        }

        robotX *= reverseMultiplier;
        robotY *= reverseMultiplier;

        // Right stick X = rotation; keep rotation sense constant so stick left always spins CCW.
        double rotation = -applySignedSquareDeadband(gamepad.right_stick_x, 0.02);
        return new Pose2d(robotX, robotY, rotation);
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

        double combined = Math.hypot(transMag, Math.abs(h));
        if (combined > 1.0) {
            double scale = 1.0 / combined;
            x *= scale;
            y *= scale;
            h *= scale;
        }

        return new Pose2d(x, y, h);
    }

    private double applySignedSquareDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return Math.copySign(scaled * scaled, value);
    }

    private double applySignedLinearDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return Math.copySign(scaled, value);
    }

    private static double wrapDeltaTicks(double delta, double period) {
        return delta - period * Math.round(delta / period);
    }

    private static double normalizeTicks(double ticks, double period) {
        return wrapDeltaTicks(ticks, period);
    }

    private void loadToroidZeroEstimate() {
        toroidZeroFreshForTeleop = false;
        toroidZeroSavedTimestampMs = 0L;
        try {
            String content = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(TOROID_ZERO_FILE_NAME));
            String[] lines = content.split("\n");
            if (lines.length >= 1) {
                toroidZeroTicks = normalizeTicks(Double.parseDouble(lines[0]), TOROID_TICKS_PER_REV);
            }
            if (lines.length >= 2) {
                toroidZeroConfidence = clamp01(Double.parseDouble(lines[1]));
            }
            if (lines.length >= 3) {
                toroidZeroSavedTimestampMs = Long.parseLong(lines[2].trim());
                toroidZeroFreshForTeleop = isToroidZeroFreshNow();
            }
        } catch (Exception e) {
            toroidZeroConfidence = 0.0;
        }
        toroidZeroInitialized = true;
    }

    private boolean isToroidZeroFreshNow() {
        if (toroidZeroSavedTimestampMs <= 0L) {
            return false;
        }
        long ageMillis = Math.max(0L, System.currentTimeMillis() - toroidZeroSavedTimestampMs);
        return ageMillis <= TOROID_ZERO_FRESHNESS_MILLIS;
    }

    private void saveToroidZeroEstimate() {
        try {
            toroidZeroSavedTimestampMs = System.currentTimeMillis();
            toroidZeroFreshForTeleop = true;
            String data = toroidZeroTicks + "\n" + toroidZeroConfidence + "\n" + toroidZeroSavedTimestampMs;
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(TOROID_ZERO_FILE_NAME), data);
        } catch (Exception e) {
            Log.w(TAG, "Failed to save toroid zero: " + e.getMessage());
        }
    }

    private void updateToroidZeroing() {
        if (toroidSensor == null || toroidMotor == null) {
            return;
        }
        // Magnetic switch homing is authoritative when present.
        if (paddleLimitSwitch != null) {
            return;
        }

        double dt = toroidZeroUpdateTimer.seconds();
        toroidZeroUpdateTimer.reset();
        if (dt > 0.0) {
            toroidZeroConfidence = clamp01(toroidZeroConfidence * Math.exp(-dt / TOROID_ZERO_CONFIDENCE_DECAY_SEC));
        }
        if (toroidTransitionPhase != TransitionPhase.NONE || toroidJamPhase != ToroidJamPhase.NONE) {
            return;
        }
        if (Math.abs(toroidMotor.getVelocity()) > TOROID_ZERO_MAX_TPS) {
            return;
        }

        int colorReadingMaxInt = 2 << 11;
        double red = (double) toroidSensor.red() / colorReadingMaxInt;
        double green = (double) toroidSensor.green() / colorReadingMaxInt;
        double blue = (double) toroidSensor.blue() / colorReadingMaxInt;
        double proximityInches = toroidSensor.getDistance(DistanceUnit.INCH);

        double greenMatch = greenResonance(red, green, blue);
        double purpleMatch = purpleResonance(red, green, blue);
        double ballGreenPresence = colorPresence(
                proximityInches, greenMatch,
                TOROID_BALL_PROX_THRESHOLD_IN, GREEN_MATCH_THRESHOLD,
                TOROID_BALL_PROX_SOFT_IN, TOROID_BALL_MATCH_SOFT
        );
        double ballPurplePresence = colorPresence(
                proximityInches, purpleMatch,
                TOROID_BALL_PROX_THRESHOLD_IN, PURPLE_MATCH_THRESHOLD,
                TOROID_BALL_PROX_SOFT_IN, TOROID_BALL_MATCH_SOFT
        );
        double ballPresence = Math.max(ballGreenPresence, ballPurplePresence);

        double whiteBias = whiteBias(red, green, blue);
        double paddleProx = belowThreshold(proximityInches, TOROID_PADDLE_PROX_THRESHOLD_IN, TOROID_PADDLE_PROX_SOFT_IN);
        double paddleWhite = aboveThreshold(whiteBias, TOROID_PADDLE_WHITE_THRESHOLD, TOROID_PADDLE_WHITE_SOFT);
        double paddlePresence = clamp01(paddleProx * paddleWhite);

        boolean paddleCandidate = ballPresence <= TOROID_BALL_SUPPRESS_THRESHOLD;
        boolean paddleSeenNow;
        if (!toroidPaddleSeen) {
            paddleSeenNow = paddleCandidate && paddlePresence >= TOROID_PADDLE_ENTER_THRESHOLD;
        } else {
            paddleSeenNow = paddleCandidate && paddlePresence >= TOROID_PADDLE_EXIT_THRESHOLD;
        }

        boolean risingEdge = !toroidPaddleSeen && paddleSeenNow;
        toroidPaddleSeen = paddleSeenNow;

        if (risingEdge) {
            int currentPosition = toroidMotor.getCurrentPosition();
            if (Double.isNaN(toroidLastPaddleObsTicks)
                    || Math.abs(wrapDeltaTicks(currentPosition - toroidLastPaddleObsTicks, TOROID_TICKS_PER_REV))
                    >= TOROID_PADDLE_MIN_SEPARATION_TICKS) {
                double sensorOffsetTicks = TOROID_TICKS_PER_REV * (TOROID_PADDLE_SENSOR_OFFSET_DEG / 360.0);
                double obsZero = currentPosition - sensorOffsetTicks;
                double delta = wrapDeltaTicks(obsZero - toroidZeroTicks, TOROID_TICKS_PER_REV);
                toroidZeroTicks = normalizeTicks(toroidZeroTicks + delta * TOROID_ZERO_BLEND, TOROID_TICKS_PER_REV);
                toroidZeroConfidence = clamp01(toroidZeroConfidence + (paddlePresence * TOROID_ZERO_CONFIDENCE_GAIN));
                toroidLastPaddleObsTicks = currentPosition;
            }
        }

        if (toroidZeroSaveTimer.seconds() >= TOROID_ZERO_SAVE_INTERVAL_SEC
                && toroidZeroConfidence >= TOROID_ZERO_MIN_SAVE_CONFIDENCE) {
            saveToroidZeroEstimate();
            toroidZeroSaveTimer.reset();
        }

        toroidLastProximityInches = proximityInches;
        toroidLastWhiteBias = whiteBias;
        toroidLastBallPresence = ballPresence;
        toroidLastPaddlePresence = paddlePresence;
    }

    private void updateToroidControl() {
        if (toroidMotor == null) {
            return;
        }

        if (!toroidZeroInitialized) {
            toroidZeroTicks = toroidMotor.getCurrentPosition();
            toroidZeroInitialized = true;
        }
        updateToroidZeroing();

        int currentPosition = toroidMotor.getCurrentPosition();
        updatePaddleLimitSwitchState();
        if (paddleLimitPressedStable && !paddleLimitPressedStablePrev) {
            captureToroidHomeFromLimitSwitch(currentPosition);
        }
        paddleLimitPressedStablePrev = paddleLimitPressedStable;

        if (isAutonomous && toroidAutoActive) {
            updateToroidAutoControl();
            return;
        }

        toroidTargetStepIndex = (int) Math.round((currentPosition - toroidZeroTicks) / TOROID_STEP_TICKS);

        // Unify toroid baseline behavior across modes:
        // teleop follows gamepad stick, autonomous runs neutral unless an auto toroid action is active.
        double stick = isAutonomous ? 0.0 : -gamepad2.right_stick_y;
        double command = applySignedLinearDeadband(stick, TOROID_JOYSTICK_DEADBAND);
        double targetRpm = command * TOROID_TRANSIT_MAX_RPM;
        boolean paddleLimitAvailable = paddleLimitSwitch != null;
        if (Math.abs(command) <= 1e-3 && isAutonomous && !autonomousToroidHomingEnabled) {
            // Keep paddle parked where autonomous initialized it until the first shoot action begins.
            targetRpm = 0.0;
        } else if (Math.abs(command) <= 1e-3 && paddleLimitAvailable && !paddleLimitSwitchSeen) {
            if (!isAutonomous) {
                // In teleop with no trusted mag reference yet, do not auto-seek.
                targetRpm = 0.0;
                toroidMode = ToroidMode.STOP;
            } else {
            double errorTicks = wrapDeltaTicks(toroidZeroTicks - currentPosition, TOROID_TICKS_PER_REV);
            double errorAbs = Math.abs(errorTicks);
            boolean hasReliableHomeEstimate = toroidZeroConfidence >= TOROID_ZERO_MIN_SAVE_CONFIDENCE;
            int transitSign = getToroidDirectionSign(TOROID_TRANSIT_CCW);
            if (hasReliableHomeEstimate && errorAbs <= TOROID_HOME_HOLD_TOLERANCE_TICKS) {
                targetRpm = 0.0;
            } else if (hasReliableHomeEstimate && errorAbs <= TOROID_HOME_DIRECT_THRESHOLD_TICKS) {
                int homeSign = errorTicks == 0.0 ? 0 : (errorTicks > 0.0 ? 1 : -1);
                targetRpm = TOROID_LIMIT_SEEK_RPM * homeSign;
            } else {
                // Large correction (or weak estimate): continue around in transit direction until home is found.
                targetRpm = TOROID_LIMIT_SEEK_RPM * transitSign;
            }
            }
        } else if (Math.abs(command) <= 1e-3 && paddleLimitAvailable && paddleLimitSwitchSeen) {
            double errorTicks = wrapDeltaTicks(toroidZeroTicks - currentPosition, TOROID_TICKS_PER_REV);
            double errorAbs = Math.abs(errorTicks);
            if (errorAbs <= TOROID_HOME_HOLD_TOLERANCE_TICKS) {
                targetRpm = 0.0;
            } else {
                targetRpm = Range.clip(
                        errorTicks * TOROID_HOME_HOLD_KP_RPM_PER_TICK,
                        -TOROID_HOME_HOLD_MAX_RPM,
                        TOROID_HOME_HOLD_MAX_RPM
                );
            }
        } else if (Math.abs(command) <= 1e-3 && TOROID_IDLE_JOSTLE_ENABLED && !isAutonomous) {
            double onSec = Math.max(0.0, TOROID_IDLE_JOSTLE_ON_SEC);
            double offSec = Math.max(0.0, TOROID_IDLE_JOSTLE_OFF_SEC);
            double cycleSec = onSec + offSec;
            if (cycleSec <= 1e-6) {
                cycleSec = 0.5;
            }
            if (toroidIdleTimer.seconds() >= cycleSec) {
                toroidIdleSign = -toroidIdleSign;
                toroidIdleTimer.reset();
            }
            if (toroidIdleTimer.seconds() <= onSec) {
                targetRpm = TOROID_IDLE_JOSTLE_RPM * toroidIdleSign;
            } else {
                targetRpm = 0.0;
            }
        } else {
            toroidIdleTimer.reset();
        }
        toroidTargetRpm = targetRpm;
        toroidTargetTps = rpmToTicksPerSec(targetRpm, TOROID_TICKS_PER_REV);

        if (Math.abs(command) > 1e-3) {
            toroidMode = ToroidMode.TRANSIT;
        } else if (Math.abs(targetRpm) > 1e-3) {
            toroidMode = ToroidMode.IDLE;
        } else {
            toroidMode = ToroidMode.STOP;
        }
        toroidTransitionPhase = TransitionPhase.NONE;
        toroidJamPhase = ToroidJamPhase.NONE;
        toroidJamConditionActive = false;

        if (toroidMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            toroidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (Math.abs(targetRpm) <= 1e-3) {
            toroidMotor.setPower(0.0);
        } else {
            toroidMotor.setVelocity(toroidTargetTps);
        }

        toroidLastApproachScale = 1.0;
        toroidLastApproachRpm = Math.abs(targetRpm);
        toroidPendingTargetRpm = targetRpm;
        toroidPendingTargetSign = targetRpm == 0.0 ? 0 : (targetRpm > 0.0 ? 1 : -1);
        toroidLastTargetSign = toroidPendingTargetSign;
        toroidLastErrorAbsTicks = 0.0;
    }

    private void startToroidAutoShootRevolutions(double revolutions, double timeoutMillis, int expectedShots) {
        if (toroidMotor == null) {
            toroidAutoActive = false;
            return;
        }
        if (!toroidZeroInitialized) {
            toroidZeroTicks = toroidMotor.getCurrentPosition();
            toroidZeroInitialized = true;
        }
        int currentPosition = toroidMotor.getCurrentPosition();
        int shootSign = getToroidDirectionSign(TOROID_SHOOT_CCW);
        double deltaTicks = Math.max(0.0, revolutions) * TOROID_TICKS_PER_REV * shootSign;
        toroidAutoTargetTicks = currentPosition + deltaTicks;
        toroidAutoDirectionSign = shootSign;
        toroidAutoTimeoutMillis = Math.max(0.0, timeoutMillis);
        toroidAutoJamAccumulatedMillis = 0.0;
        toroidAutoExpectedShots = Math.max(0, expectedShots);
        toroidAutoDetectedShots = 0;
        toroidAutoWheelCurrentBaselineA = Double.NaN;
        toroidAutoShotCandidatePrev = false;
        toroidShotDetectCommandedSinceNs = Long.MIN_VALUE;
        toroidAutoTimer.reset();
        toroidAutoLoopTimer.reset();
        toroidAutoActive = true;
    }

    private void updateToroidShotDetection() {
        if (wheel == null) {
            toroidAutoShotCandidatePrev = false;
            toroidShotDetectCommandedSinceNs = Long.MIN_VALUE;
            return;
        }
        if (toroidJamPhase != ToroidJamPhase.NONE) {
            toroidAutoShotCandidatePrev = false;
            return;
        }
        long nowNs = System.nanoTime();
        double desiredTpsSigned = shooterDesiredWheelTicksPerSecond;
        double desiredTps = Math.abs(desiredTpsSigned);
        boolean shooterCommanded = shooterEnabled && desiredTpsSigned > 1e-3;
        if (!shooterCommanded) {
            toroidShotDetectCommandedSinceNs = Long.MIN_VALUE;
            toroidAutoShotCandidatePrev = false;
            return;
        }
        if (toroidShotDetectCommandedSinceNs == Long.MIN_VALUE) {
            toroidShotDetectCommandedSinceNs = nowNs;
        }
        if ((nowNs - toroidShotDetectCommandedSinceNs) < (long) (TOROID_SHOT_DETECT_ARM_MILLIS * 1_000_000.0)) {
            toroidAutoShotCandidatePrev = false;
            return;
        }

        double currentA = wheel.getCurrent(CurrentUnit.AMPS);
        double actualTps = Math.abs(wheel.getVelocity());
        if (desiredTps <= 1e-3 || Double.isNaN(currentA) || Double.isNaN(actualTps)) {
            toroidAutoShotCandidatePrev = false;
            return;
        }
        if (Double.isNaN(toroidAutoWheelCurrentBaselineA)) {
            toroidAutoWheelCurrentBaselineA = currentA;
        }

        boolean velocityDroop = actualTps <= desiredTps * (1.0 - TOROID_SHOT_DETECT_VELOCITY_DROP_RATIO);
        double currentDeltaA = currentA - toroidAutoWheelCurrentBaselineA;
        boolean currentSpike = currentDeltaA >= TOROID_SHOT_DETECT_CURRENT_SPIKE_AMPS;
        boolean candidate = velocityDroop && currentSpike;
        boolean risingEdge = candidate && !toroidAutoShotCandidatePrev;
        toroidAutoShotCandidatePrev = candidate;

        if (!candidate) {
            toroidAutoWheelCurrentBaselineA +=
                    (currentA - toroidAutoWheelCurrentBaselineA) * TOROID_SHOT_DETECT_BASELINE_ALPHA;
        }

        if (!risingEdge) {
            return;
        }
        boolean refractoryElapsed = toroidLastShotDetectTimeNs == Long.MIN_VALUE
                || (nowNs - toroidLastShotDetectTimeNs) >= (long) (TOROID_SHOT_DETECT_REFRACTORY_MILLIS * 1_000_000.0);
        if (!refractoryElapsed) {
            return;
        }

        toroidAutoDetectedShots++;
        toroidLastShotDetectTimeNs = nowNs;
    }

    private void updateToroidAutoControl() {
        if (toroidMotor == null) {
            toroidAutoActive = false;
            return;
        }

        double loopDtMillis = toroidAutoLoopTimer.milliseconds();
        toroidAutoLoopTimer.reset();
        if (toroidJamPhase != ToroidJamPhase.NONE) {
            toroidAutoJamAccumulatedMillis += Math.max(0.0, loopDtMillis);
        }
        int currentPosition = toroidMotor.getCurrentPosition();
        double targetTicks = toroidAutoTargetTicks;
        double errorTicks = targetTicks - currentPosition;
        double errorAbs = Math.abs(errorTicks);
        toroidTargetStepIndex = (int) Math.round((targetTicks - toroidZeroTicks) / TOROID_STEP_TICKS);

        double effectiveElapsedMillis = Math.max(0.0, toroidAutoTimer.milliseconds() - toroidAutoJamAccumulatedMillis);
        boolean timedOut = toroidAutoTimeoutMillis > 0.0
                && effectiveElapsedMillis >= toroidAutoTimeoutMillis;
        boolean reachedExpectedShots = toroidAutoExpectedShots > 0 && toroidAutoDetectedShots >= toroidAutoExpectedShots;
        if (errorAbs <= TOROID_POSITION_TOLERANCE_TICKS || timedOut || reachedExpectedShots) {
            toroidAutoActive = false;
            toroidTargetRpm = 0.0;
            toroidTargetTps = 0.0;
            toroidMode = ToroidMode.STOP;
            toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            toroidMotor.setPower(0.0);
            return;
        }

        double speedRpm = errorAbs <= TOROID_APPROACH_START_TICKS ? TOROID_APPROACH_RPM : TOROID_SHOOT_RPM;
        double directionSign = toroidAutoDirectionSign != 0 ? toroidAutoDirectionSign : Math.signum(errorTicks);
        double targetRpm = speedRpm * directionSign;
        toroidTargetRpm = targetRpm;
        toroidTargetTps = rpmToTicksPerSec(targetRpm, TOROID_TICKS_PER_REV);

        if (updateToroidJamControl(targetRpm)) {
            toroidMode = ToroidMode.SHOOT;
            toroidTransitionPhase = TransitionPhase.NONE;
            return;
        }

        toroidMode = ToroidMode.SHOOT;
        toroidTransitionPhase = TransitionPhase.NONE;

        if (toroidMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            toroidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toroidMotor.setVelocity(toroidTargetTps);
    }

    private boolean updateToroidJamControl(double targetRpm) {
        double actualTps = toroidMotor.getVelocity();
        double currentAmps = toroidMotor.getCurrent(CurrentUnit.AMPS);
        boolean jamAllowed = Math.abs(targetRpm) > 1e-3
                && toroidJamCooldownTimer.seconds() >= TOROID_JAM_COOLDOWN_SEC;
        boolean jamCondition = jamAllowed
                && Math.abs(actualTps) <= TOROID_JAM_SPEED_TPS
                && currentAmps >= TOROID_JAM_CURRENT_AMPS;

        if (toroidJamPhase == ToroidJamPhase.NONE) {
            if (jamCondition) {
                if (!toroidJamConditionActive) {
                    toroidJamConditionActive = true;
                    toroidJamDetectTimer.reset();
                }
                if (toroidJamDetectTimer.seconds() >= TOROID_JAM_DETECT_SEC) {
                    toroidJamPhase = ToroidJamPhase.RELAX;
                    toroidJamTimer.reset();
                    toroidJamResumeRpm = targetRpm;
                    toroidJamResumeSign = targetRpm == 0.0 ? 0 : (targetRpm > 0.0 ? 1 : -1);
                    toroidJamConditionActive = false;
                    toroidJamDetectTimer.reset();
                } else {
                    return false;
                }
            } else {
                toroidJamConditionActive = false;
                toroidJamDetectTimer.reset();
                return false;
            }
        }

        if (toroidMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            toroidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (toroidJamPhase == ToroidJamPhase.RELAX) {
            toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            toroidMotor.setPower(0.0);
            if (Math.abs(actualTps) <= TOROID_JAM_STOP_TPS
                    || toroidJamTimer.seconds() >= TOROID_JAM_RELAX_MAX_SEC) {
                toroidJamPhase = ToroidJamPhase.REVERSE;
                toroidJamTimer.reset();
            }
            return true;
        }

        if (toroidJamPhase == ToroidJamPhase.REVERSE) {
            toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            int reverseSign = toroidJamResumeSign != 0 ? -toroidJamResumeSign : -1;
            double reverseRpm = TOROID_JAM_REVERSE_RPM * reverseSign;
            toroidMotor.setVelocity(rpmToTicksPerSec(reverseRpm, TOROID_TICKS_PER_REV));
            if (toroidJamTimer.seconds() >= TOROID_JAM_REVERSE_SEC) {
                toroidJamPhase = ToroidJamPhase.FORWARD;
                toroidJamTimer.reset();
            }
            return true;
        }

        if (toroidJamPhase == ToroidJamPhase.FORWARD) {
            toroidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            int forwardSign = toroidJamResumeSign != 0 ? toroidJamResumeSign : 1;
            double forwardRpm = TOROID_JAM_FORWARD_RPM * forwardSign;
            toroidMotor.setVelocity(rpmToTicksPerSec(forwardRpm, TOROID_TICKS_PER_REV));
            if (toroidJamTimer.seconds() >= TOROID_JAM_FORWARD_SEC) {
                toroidJamPhase = ToroidJamPhase.NONE;
                toroidJamCooldownTimer.reset();
            }
            return true;
        }

        return false;
    }

    private int getToroidDirectionSign(boolean ccw) {
        int sign = TOROID_CCW_IS_POSITIVE ? 1 : -1;
        if (!ccw) {
            sign = -sign;
        }
        return sign;
    }

    private void updatePaddleLimitSwitchState() {
        boolean rawPressed = paddleLimitSwitch != null && paddleLimitSwitch.isPressed();
        paddleLimitPressedRaw = rawPressed;
        paddleLimitPressedStable = rawPressed;
    }

    private void captureToroidHomeFromLimitSwitch(int currentPositionTicks) {
        paddleLimitCaptureTicks = currentPositionTicks;
        double offsetTicks = TOROID_TICKS_PER_REV * (TOROID_HOME_OFFSET_DEGREES / 360.0);
        toroidZeroTicks = normalizeTicks(currentPositionTicks - offsetTicks, TOROID_TICKS_PER_REV);
        toroidZeroConfidence = 1.0;
        toroidZeroInitialized = true;
        paddleLimitSwitchSeen = true;
        saveToroidZeroEstimate();
    }

    private static double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    private double getShooterHeadingError() {
        Pose2d basePose = latestPoseEstimate != null ? latestPoseEstimate : lastAprilTagFieldPosition;
        Vector2d basket = getActiveBasketPosition();
        if (basePose == null || basket == null) {
            return 0.0;
        }
        double desiredHeading = getAimHeadingFromRobotPose(basePose, basket);
        return Angle.normDelta(desiredHeading - basePose.getHeading());
    }

    private enum ToroidMode {
        TRANSIT,
        SHOOT,
        IDLE,
        STOP
    }

    private enum ToroidJamPhase {
        NONE,
        RELAX,
        REVERSE,
        FORWARD
    }

    private enum TransitionPhase {
        NONE,
        COAST,
        BRAKE
    }

    private enum IntakeState {
        FORWARD,
        REVERSE_REJECT,
        OFF
    }

    private enum IntakeJamPhase {
        NONE,
        RELAX,
        REVERSE
    }

    private enum ObeliskPattern {
        UNKNOWN,
        GREEN_FIRST,
        GREEN_MIDDLE,
        GREEN_LAST
    }

    private void updateAutonomousMechanisms(double dtMillis) {
        updateToroidControl();
        long nowMs = System.currentTimeMillis();
        if (autonomousIntakeHoldUntilMs > 0L && nowMs >= autonomousIntakeHoldUntilMs) {
            autonomousIntakePower = 0.0;
            autonomousIntakeHoldUntilMs = 0L;
        }

        if (autonomousIntakePower > 0.0) {
            intakeState = IntakeState.FORWARD;
        } else if (autonomousIntakePower < 0.0) {
            intakeState = IntakeState.REVERSE_REJECT;
        } else {
            intakeState = IntakeState.OFF;
        }
        double intakePowerTarget = autonomousIntakePower;
        if (toroidJamPhase != ToroidJamPhase.NONE) {
            intakeState = IntakeState.OFF;
            intakePowerTarget = 0.0;
        }
        updateIntakePower(intakePowerTarget, dtMillis);
        lift.setPosition(lifted ? 0.0 : 1.0);
    }

    private boolean maybeStartCommandActions(OpModeCommand command) {
        if (command == null) return true;

        boolean actionsStarted = true;

        if (command.ShooterEnabled != null) {
            shooterEnabled = command.ShooterEnabled;
        }

        if (command.IntakePower != null) {
            autonomousIntakePower = command.IntakePower;
            if (Math.abs(autonomousIntakePower) <= 1e-3) {
                autonomousIntakeHoldUntilMs = 0L;
            }
        }
        if (command.IntakeHoldMillis != null && command.IntakePower != null && Math.abs(command.IntakePower) > 1e-3) {
            autonomousIntakeHoldUntilMs = System.currentTimeMillis() + Math.max(0L, Math.round(command.IntakeHoldMillis));
        }

        if (command.ToroidShootRevolutions != null) {
            double timeoutMillis = command.ToroidTimeoutMillis != null ? command.ToroidTimeoutMillis : 0.0;
            int expectedShots = command.ToroidExpectedShots != null ? command.ToroidExpectedShots : 0;
            // After first autonomous shoot command, allow normal home-seek/hold behavior between actions.
            autonomousToroidHomingEnabled = true;
            startToroidAutoShootRevolutions(command.ToroidShootRevolutions, timeoutMillis, expectedShots);
        }

        return actionsStarted;
    }

    private OpModeState evaluateCommandSequence(double dtMillis) {
        if (commandSequence.isEmpty()) {
            currentAutoDriveTarget = null;
            autonomousIntakePower = 0.0;
            autonomousIntakeHoldUntilMs = 0L;
            updateAutonomousMechanisms(dtMillis);
            updateShooterControl(false);
            OpModeState _continuationState = continuationState;
            continuationState = null;
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            currentCommandActionsStarted = false;
            return _continuationState == null ? OpModeState.STOPPED_UNTIL_END : _continuationState;
        }

        if (currentCommand == null) {
            currentCommand = commandSequence.get(0);
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            currentCommandActionsStarted = false;
        }

        updateAutonomousMechanisms(dtMillis);

        int waitUntilMillis = currentCommand.WaitUntilElapsedMillis == null ? 0 : currentCommand.WaitUntilElapsedMillis;
        if (timeSinceStart.milliseconds() < waitUntilMillis) {
            setDrivePower(new Pose2d());
            updateShooterControl(false);
            return OpModeState.COMMAND_SEQUENCE;
        }

        if (!currentCommandActionsStarted) {
            currentCommandActionsStarted = maybeStartCommandActions(currentCommand);
        }

        updateShooterControl(false);

        currentAutoDriveTarget = currentCommand.DriveToPose;
        if (currentCommand.DriveToPose != null) {
            Pose2d drivePower = getPoseTargetAutoDriveControl(currentCommand.DriveToPose);
            if (currentCommand.DrivePowerScale != null) {
                double scale = currentCommand.DrivePowerScale;
                drivePower = new Pose2d(drivePower.getX() * scale, drivePower.getY() * scale, drivePower.getHeading());
            }
            setDrivePower(drivePower);
        } else {
            setDrivePower(new Pose2d());
        }

        boolean driveCompleted = currentCommand.DriveToPose == null || isAtPoseTarget(currentCommand.DriveToPose, currentCommand.DriveSettleThresholdRatio);
        boolean settledRightNow = driveCompleted;

        if (currentCommand.RequireActionStarted) {
            settledRightNow = settledRightNow && currentCommandActionsStarted;
        }
        if (currentCommand.RequireShooterEnabled && currentCommand.ShooterEnabled != null) {
            settledRightNow = settledRightNow && (shooterEnabled == currentCommand.ShooterEnabled);
        }
        if (currentCommand.ToroidShootRevolutions != null) {
            settledRightNow = settledRightNow && !toroidAutoActive;
        }

        boolean minTimeElapsed = currentCommandTime.milliseconds() > currentCommand.MinTimeMillis;
        boolean commandCompleted = settledRightNow && minTimeElapsed && currentCommandSettledTime.milliseconds() > currentCommand.SettleTimeMillis;
        boolean debugAdvance = !debugMode || gamepad1.a;
        if (commandCompleted && debugAdvance) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            setDrivePower(new Pose2d());
            commandSequence.remove(0);
            currentCommand = null;
            currentCommandActionsStarted = false;
            currentCommandSettledTime.reset();
        } else if (!settledRightNow) {
            currentCommandSettledTime.reset();
        }

        return OpModeState.COMMAND_SEQUENCE;
    }

    private void evaluatePositioningSystems() {
        if (isAutonomous) {
            goalTagVisible = false;
            tagPoseQueue.clear();
            return;
        }

        LLStatus status = limelight.getStatus();
        if (status != null) {
            limelightName = status.getName();
            limelightPipelineIndex = status.getPipelineIndex();
            limelightPipelineType = status.getPipelineType();
            limelightFps = status.getFps();
            limelightTempC = status.getTemp();
            limelightCpu = status.getCpu();
        }

        LLResult result = limelight.getLatestResult();
        limelightResultValid = result != null && result.isValid();
        limelightCaptureLatencyMs = result != null ? result.getCaptureLatency() : Double.NaN;
        limelightTargetingLatencyMs = result != null ? result.getTargetingLatency() : Double.NaN;
        limelightParseLatencyMs = result != null ? result.getParseLatency() : Double.NaN;
        limelightStalenessMs = result != null ? result.getStaleness() : Double.NaN;
        limelightTx = result != null ? result.getTx() : Double.NaN;
        limelightTy = result != null ? result.getTy() : Double.NaN;
        limelightRawPoseX = Double.NaN;
        limelightRawPoseY = Double.NaN;
        limelightRawPoseHeadingDeg = Double.NaN;
        limelightRawPoseSource = "none";
        limelightRejectedDefaultPose = false;
        limelightBotposeString = "";
        limelightVisionConfidence = 0.0;
        List<LLResultTypes.FiducialResult> detections = result != null ? result.getFiducialResults() : null;
        boolean aprilTagPipelineActive = isAprilTagPipelineActive();
        limelightAprilTagPipelineActive = aprilTagPipelineActive;
        boolean resultIsFresh = result != null && result.isValid() &&
                !Double.isNaN(result.getStaleness()) &&
                result.getStaleness() <= LIMELIGHT_STALE_THRESHOLD_MS;
        boolean canProcessVision = aprilTagPipelineActive && resultIsFresh &&
                detections != null && !detections.isEmpty();

        goalTagVisible = false;
        lastTagDetectionsCount = detections != null ? detections.size() : 0;
        if (detections != null && !detections.isEmpty()) {
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult detection : detections) {
                if (ids.length() > 0) {
                    ids.append(',');
                }
                ids.append(detection.getFiducialId());
            }
            lastTagDetectionIds = ids.toString();
        } else {
            lastTagDetectionIds = "";
        }
        LLResultTypes.FiducialResult bestDetection = null;
        Pose2d bestPose = null;
        Pose2d limelightRawBotPose = result != null ? limelightPoseToRoadRunner(result.getBotpose()) : null;
        if (result != null && result.getBotpose() != null) {
            limelightBotposeString = result.getBotpose().toString();
        }
        if (detections != null) {
            for (LLResultTypes.FiducialResult detection : detections) {
                maybeUpdateObeliskPattern(detection);
                int detectionId = detection.getFiducialId();
                if (isGoalAprilTag(detectionId)) {
                    goalTagVisible = true;
                }

                if (!isAllowedAprilTag(detectionId)) {
                    continue;
                }

                Pose2d detectionPose = limelightPoseToRoadRunner(detection.getRobotPoseFieldSpace());
                if (detectionPose == null) {
                    continue;
                }
                if (isLikelyDefaultZeroPose(detectionPose, detection.getTargetXDegrees(), detection.getTargetYDegrees())) {
                    limelightRejectedDefaultPose = true;
                    continue;
                }

                if (Double.isNaN(limelightRawPoseX)) {
                    limelightRawPoseX = detectionPose.getX();
                    limelightRawPoseY = detectionPose.getY();
                    limelightRawPoseHeadingDeg = Math.toDegrees(detectionPose.getHeading());
                    limelightRawPoseSource = "fiducial";
                }
                detectionPose = compensateVisionPoseToNow(detectionPose, result);

                double rangeInches = getFiducialRangeInches(detection);
                double bearingRadians = Math.toRadians(detection.getTargetXDegrees());
                if (Double.isNaN(rangeInches) ||
                        rangeInches > APRIL_TAG_SANITY_MAX_RANGE ||
                        rangeInches < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                        Math.abs(bearingRadians) > APRIL_TAG_SANITY_MAX_BEARING) {
                    continue;
                }

                if (bestDetection == null || rangeInches < getFiducialRangeInches(bestDetection)) {
                    bestDetection = detection;
                    bestPose = detectionPose;
                }
            }
        }

        boolean canFuseVisionPose = canProcessVision && bestDetection != null && bestPose != null;
        if (canFuseVisionPose) {
            // Use one representative measurement per frame (closest trusted fiducial) to reduce multi-tag jitter.
            enqueueTagPose(bestPose);
        }

        if (Double.isNaN(limelightRawPoseX) && limelightRawBotPose != null &&
                !isLikelyDefaultZeroPose(limelightRawBotPose, limelightTx, limelightTy)) {
            limelightRawPoseX = limelightRawBotPose.getX();
            limelightRawPoseY = limelightRawBotPose.getY();
            limelightRawPoseHeadingDeg = Math.toDegrees(limelightRawBotPose.getHeading());
            limelightRawPoseSource = "botpose";
        } else if (limelightRawBotPose != null && isLikelyDefaultZeroPose(limelightRawBotPose, limelightTx, limelightTy)) {
            limelightRejectedDefaultPose = true;
        }

        if (bestDetection != null) {
            lastDetectionYaw = 0.0;
            lastDetectionBearing = Math.toRadians(bestDetection.getTargetXDegrees());
            lastDetectionElevation = Math.toRadians(bestDetection.getTargetYDegrees());
            lastDetectionRange = getFiducialRangeInches(bestDetection);
            lastTagDerivedPose = bestPose;
        } else {
            lastTagDerivedPose = null;
        }

        if (tagPoseQueue.size() >= APRIL_TAG_MIN_QUEUE_SAMPLES) {
            Pose2d averagePose = calculateAveragePose(tagPoseQueue);
            Pose2d variancePose = calculateVariancePose(tagPoseQueue, averagePose);
            boolean varianceAcceptable = variancePose.getX() <= APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD &&
                    variancePose.getY() <= APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD &&
                    variancePose.getHeading() <= APRIL_TAG_VARIANCE_HEADING_THRESHOLD;
            if (varianceAcceptable && canFuseVisionPose) {
                Pose2d odomPose = latestPoseEstimate != null ? latestPoseEstimate : drive.getPoseEstimate();
                if (odomPose != null) {
                    double confidence = computeVisionConfidence(bestDetection, variancePose, result);
                    limelightVisionConfidence = confidence;
                    Pose2d fusedPose = blendVisionIntoOdometry(odomPose, averagePose, confidence);
                    drive.setPoseEstimate(fusedPose);
                    latestPoseEstimate = fusedPose;
                    lastAprilTagFieldPosition = fusedPose;
                }
            }
        }

        if (!canFuseVisionPose) {
            tagPoseQueue.clear();
        }
    }

    private boolean isAprilTagPipelineActive() {
        if (limelightPipelineType == null) {
            return false;
        }
        String t = limelightPipelineType.toLowerCase();
        return t.contains("apriltag") || t.contains("fiducial");
    }

    private boolean isLikelyDefaultZeroPose(Pose2d pose, double txDeg, double tyDeg) {
        if (pose == null) {
            return false;
        }
        boolean nearZeroPose = Math.abs(pose.getX()) < 1e-6 &&
                Math.abs(pose.getY()) < 1e-6 &&
                Math.abs(Angle.norm(pose.getHeading())) < 1e-6;
        boolean targetClearlyOffCenter = Math.abs(txDeg) > 0.2 || Math.abs(tyDeg) > 0.2;
        return nearZeroPose && targetClearlyOffCenter;
    }

    private double computeVisionConfidence(
            LLResultTypes.FiducialResult detection,
            Pose2d variancePose,
            LLResult result) {
        if (detection == null) {
            return 0.0;
        }

        double rangeInches = getFiducialRangeInches(detection);
        double txRadians = Math.abs(Math.toRadians(detection.getTargetXDegrees()));
        double tagCountScore = clamp01(lastTagDetectionsCount / 2.0);
        double rangeScore = Double.isNaN(rangeInches) ? 0.0 :
                1.0 - clamp01((rangeInches - APRIL_TAG_RECOGNITION_MIN_RANGE) /
                        Math.max(1e-6, APRIL_TAG_RECOGNITION_MAX_RANGE - APRIL_TAG_RECOGNITION_MIN_RANGE));
        double bearingScore = 1.0 - clamp01(txRadians / APRIL_TAG_SANITY_MAX_BEARING);
        double staleness = (result != null && !Double.isNaN(result.getStaleness())) ? result.getStaleness() : LIMELIGHT_STALE_THRESHOLD_MS;
        double stalenessScore = 1.0 - clamp01(staleness / LIMELIGHT_STALE_THRESHOLD_MS);

        double varianceScore = 0.0;
        if (variancePose != null) {
            double vx = 1.0 - clamp01(variancePose.getX() / (APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD * 2.0));
            double vy = 1.0 - clamp01(variancePose.getY() / (APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD * 2.0));
            double vh = 1.0 - clamp01(variancePose.getHeading() / (APRIL_TAG_VARIANCE_HEADING_THRESHOLD * 2.0));
            varianceScore = Math.min(vx, Math.min(vy, vh));
        }

        double weighted = (0.35 * tagCountScore) +
                (0.25 * rangeScore) +
                (0.15 * bearingScore) +
                (0.10 * stalenessScore) +
                (0.15 * varianceScore);
        return clamp01(0.15 + (0.85 * weighted));
    }

    private Pose2d blendVisionIntoOdometry(Pose2d odomPose, Pose2d visionPose, double confidence) {
        if (odomPose == null || visionPose == null) {
            return odomPose != null ? odomPose : visionPose;
        }

        double dx = visionPose.getX() - odomPose.getX();
        double dy = visionPose.getY() - odomPose.getY();
        double dHeading = Angle.normDelta(visionPose.getHeading() - odomPose.getHeading());

        double correctionDistance = Math.hypot(dx, dy);
        double translationCapScale = correctionDistance > APRIL_TAG_MAX_CORRECTION_DISTANCE
                ? (APRIL_TAG_MAX_CORRECTION_DISTANCE / correctionDistance)
                : 1.0;
        double headingCapScale = Math.abs(dHeading) > APRIL_TAG_MAX_CORRECTION_HEADING
                ? (APRIL_TAG_MAX_CORRECTION_HEADING / Math.abs(dHeading))
                : 1.0;

        double translationAlpha = clamp01(APRIL_TAG_BLEND_TRANSLATION_WEIGHT * confidence) * translationCapScale;
        double headingAlpha = clamp01(APRIL_TAG_BLEND_HEADING_WEIGHT * confidence) * headingCapScale;

        return new Pose2d(
                odomPose.getX() + (dx * translationAlpha),
                odomPose.getY() + (dy * translationAlpha),
                Angle.norm(odomPose.getHeading() + (dHeading * headingAlpha)));
    }

    private void enqueueTagPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        if (tagPoseQueue.size() >= APRIL_TAG_QUEUE_CAPACITY) {
            tagPoseQueue.poll();
        }
        tagPoseQueue.offer(pose);
    }

    private Pose2d limelightPoseToRoadRunner(Pose3D pose) {
        if (pose == null || pose.getPosition() == null || pose.getOrientation() == null) {
            return null;
        }
        double x = pose.getPosition().x * METERS_TO_INCHES;
        double y = pose.getPosition().y * METERS_TO_INCHES;
        double heading = pose.getOrientation().getYaw(AngleUnit.RADIANS);
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) {
            return null;
        }
        return new Pose2d(x, y, Angle.norm(heading));
    }

    private void recordOdometryPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        TimedPose sample = new TimedPose();
        sample.timeNs = System.nanoTime();
        sample.pose = new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
        odometryPoseHistory.offer(sample);
        long cutoffNs = sample.timeNs - ODOMETRY_HISTORY_WINDOW_NS;
        while (!odometryPoseHistory.isEmpty() && odometryPoseHistory.peek().timeNs < cutoffNs) {
            odometryPoseHistory.poll();
        }
    }

    private Pose2d compensateVisionPoseToNow(Pose2d visionPose, LLResult result) {
        if (visionPose == null || result == null || latestPoseEstimate == null || odometryPoseHistory.isEmpty()) {
            return visionPose;
        }

        double latencyMs = 0.0;
        if (!Double.isNaN(result.getCaptureLatency())) latencyMs += result.getCaptureLatency();
        if (!Double.isNaN(result.getTargetingLatency())) latencyMs += result.getTargetingLatency();
        if (!Double.isNaN(result.getParseLatency())) latencyMs += result.getParseLatency();
        if (!Double.isNaN(result.getStaleness())) latencyMs += result.getStaleness();
        if (latencyMs <= 0.0) {
            return visionPose;
        }

        long visionTimestampNs = System.nanoTime() - (long) (latencyMs * 1_000_000.0);
        Pose2d odomAtVisionTime = getPoseAtTime(visionTimestampNs);
        if (odomAtVisionTime == null) {
            return visionPose;
        }

        double dx = latestPoseEstimate.getX() - odomAtVisionTime.getX();
        double dy = latestPoseEstimate.getY() - odomAtVisionTime.getY();
        double dHeading = Angle.normDelta(latestPoseEstimate.getHeading() - odomAtVisionTime.getHeading());
        return new Pose2d(
                visionPose.getX() + dx,
                visionPose.getY() + dy,
                Angle.norm(visionPose.getHeading() + dHeading));
    }

    private Pose2d getPoseAtTime(long targetNs) {
        TimedPose prev = null;
        for (TimedPose sample : odometryPoseHistory) {
            if (sample.timeNs >= targetNs) {
                if (prev == null) {
                    return sample.pose;
                }
                long span = sample.timeNs - prev.timeNs;
                if (span <= 0) {
                    return sample.pose;
                }
                double t = (double) (targetNs - prev.timeNs) / (double) span;
                t = clamp01(t);
                double x = prev.pose.getX() + (sample.pose.getX() - prev.pose.getX()) * t;
                double y = prev.pose.getY() + (sample.pose.getY() - prev.pose.getY()) * t;
                double headingDelta = Angle.normDelta(sample.pose.getHeading() - prev.pose.getHeading());
                double heading = Angle.norm(prev.pose.getHeading() + (headingDelta * t));
                return new Pose2d(x, y, heading);
            }
            prev = sample;
        }
        return odometryPoseHistory.peekLast() != null ? odometryPoseHistory.peekLast().pose : null;
    }

    private static double getFiducialRangeInches(LLResultTypes.FiducialResult detection) {
        if (detection == null || detection.getTargetPoseCameraSpace() == null ||
                detection.getTargetPoseCameraSpace().getPosition() == null) {
            return Double.NaN;
        }
        double x = detection.getTargetPoseCameraSpace().getPosition().x;
        double y = detection.getTargetPoseCameraSpace().getPosition().y;
        double z = detection.getTargetPoseCameraSpace().getPosition().z;
        double distanceMeters = Math.sqrt((x * x) + (y * y) + (z * z));
        return distanceMeters * METERS_TO_INCHES;
    }

    private boolean isAllowedAprilTag(int tagId) {
        for (int allowedId : APRIL_TAG_ALLOWED_IDS) {
            if (allowedId == tagId) return true;
        }
        return false;
    }

    private boolean isGoalAprilTag(int tagId) {
        for (int goalId : GOAL_TAG_IDS) {
            if (goalId == tagId) return true;
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

    private void maybeUpdateObeliskPattern(LLResultTypes.FiducialResult detection) {
        if (detection == null) {
            return;
        }
        int tagId = detection.getFiducialId();
        if (!isObeliskTag(tagId)) {
            return;
        }
        AprilTagMetadata tag = APRIL_TAG_LIBRARY.lookupTag(tagId);
        if (tag == null || tag.fieldPosition == null) {
            return;
        }

        lastObeliskTagId = tagId;
        lastObeliskTagHeading = getTagFieldHeading(tagId);
        lastObeliskTagX = tag.fieldPosition.get(0);

        if (lastObeliskTagX >= 0) {
            return; // must live on -X side
        }

        double headingError = Math.abs(Angle.normDelta(lastObeliskTagHeading - OBELISK_TARGET_HEADING_RADIANS));
        if (headingError > OBELISK_HEADING_TOLERANCE_RADIANS) {
            return; // not facing +X
        }

        double rangeInches = getFiducialRangeInches(detection);
        if (Double.isNaN(rangeInches)) {
            return;
        }
        double bearingRadians = Math.abs(Math.toRadians(detection.getTargetXDegrees()));
        if (rangeInches > APRIL_TAG_RECOGNITION_MAX_RANGE ||
                rangeInches < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                bearingRadians > APRIL_TAG_RECOGNITION_BEARING_THRESHOLD) {
            return;
        }

        ObeliskPattern observedPattern = patternFromTagId(tagId);
        if (observedPattern != ObeliskPattern.UNKNOWN) {
            recordObeliskVote(observedPattern);
        }
    }

    private Pose2d calculateAveragePose(Queue<Pose2d> poses) {
        double sumX = 0, sumY = 0;
        double sumHeadingSin = 0, sumHeadingCos = 0;
        for (Pose2d pose : poses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumHeadingSin += Math.sin(pose.getHeading());
            sumHeadingCos += Math.cos(pose.getHeading());
        }
        int count = poses.size();
        double avgHeading = Math.atan2(sumHeadingSin / count, sumHeadingCos / count);
        return new Pose2d(sumX / count, sumY / count, Angle.norm(avgHeading));
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
        if (latestPoseEstimate == null || error == null) {
            autoStrafePriorityActive = false;
            return new Pose2d();
        }

        double distance = Math.hypot(error.getX(), error.getY());
        double robotHeading = latestPoseEstimate.getHeading();
        double headingToError = Math.atan2(error.getY(), error.getX()) - robotHeading;
        double xErr = distance * Math.cos(headingToError);
        double yErr = distance * Math.sin(headingToError);
        boolean xErrEliminated = Math.abs(xErr) < 0.75;
        boolean yErrEliminated = Math.abs(yErr) < 0.75;
        // Keep turn deadband tighter than the most precise settle threshold to avoid stalling.
        boolean thetaErrEliminated = Math.abs(error.getHeading()) < (TURN_ERROR_THRESHOLD * AUTO_TURN_DEADBAND_RATIO);
        boolean strafePriority = distance <= AUTO_STRAFE_PRIORITY_DISTANCE_INCHES
                && !yErrEliminated
                && Math.abs(yErr) >= (Math.abs(xErr) * AUTO_STRAFE_PRIORITY_AXIS_RATIO);
        autoStrafePriorityActive = strafePriority;

        double xMin = xErrEliminated ? 0 : Math.signum(xErr) * AUTO_MIN_TRANSLATION_POWER;
        double yMin = yErrEliminated ? 0 : Math.signum(yErr) * AUTO_MIN_TRANSLATION_POWER;
        double thetaMin = (thetaErrEliminated || strafePriority) ? 0 : Math.signum(error.getHeading()) * AUTO_MIN_ROTATION_POWER;

        double translationGain = SPEED_GAIN * AUTO_TRANSLATION_GAIN_SCALE;
        double x = Math.abs(xMin) > Math.abs(xErr * translationGain) ? xMin : xErr * translationGain;
        double y = Math.abs(yMin) > Math.abs(yErr * translationGain) ? yMin : yErr * translationGain;
        double theta = Math.abs(thetaMin) > Math.abs(error.getHeading() * TURN_GAIN) ? thetaMin : error.getHeading() * TURN_GAIN;
        if (strafePriority) {
            theta *= AUTO_STRAFE_PRIORITY_TURN_SCALE;
        }

        double xPower = Range.clip(x, -1, 1);
        double yPower = Range.clip(y, -1, 1);
        double hPower = Range.clip(theta, -1, 1);

        return new Pose2d(xPower, yPower, hPower);
    }

    double lastDetectionYaw = 0.0;
    double lastDetectionBearing = 0.0;
    double lastDetectionElevation = 0.0;
    double lastDetectionRange = 0.0;

    private static double getTagFieldHeading(int tagId) {
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

    private static Pose2d mirrorPoseForBlue(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Angle.norm(-pose.getHeading()));
    }

    private static Pose2d getGoalStartPose(AllianceColor alliance) {
        int tagId = alliance == AllianceColor.RED ? GOAL_RED_START_TAG_ID : GOAL_BLUE_START_TAG_ID;
        double heading = Angle.norm(getTagFieldHeading(tagId) + Math.PI);
        double y = alliance == AllianceColor.RED ? GOAL_START_Y : -GOAL_START_Y;
        return new Pose2d(GOAL_START_X, y, heading);
    }

    private static Pose2d getAudienceStartPose(AllianceColor alliance) {
        Pose2d base = new Pose2d(AUDIENCE_START_X, AUDIENCE_START_Y, AUDIENCE_START_HEADING);
        return alliance == AllianceColor.RED ? base : mirrorPoseForBlue(base);
    }

    private static Pose2d getLeaveTargetPose(AllianceColor alliance) {
        Pose2d start = getAudienceStartPose(alliance);
        return applyRobotRelativeOffset(start, AUDIENCE_LEAVE_FORWARD_INCHES, 0.0);
    }

    private Pose2d getGoalStartLeavePose(AllianceColor alliance) {
        Pose2d redShootPose = getBackShootPose(AllianceColor.RED);
        Pose2d redLeavePose = new Pose2d(
                redShootPose.getX() + 12.0,
                redShootPose.getY() + 12.0,
                redShootPose.getHeading()
        );
        return alliance == AllianceColor.RED ? redLeavePose : mirrorPoseForBlue(redLeavePose);
    }

    private static Pose2d getBackParkPose(AllianceColor alliance) {
        Pose2d base = new Pose2d(BACK_PARK_X, BACK_PARK_Y, BACK_PARK_HEADING);
        return alliance == AllianceColor.RED ? base : mirrorPoseForBlue(base);
    }

    private Pose2d getBackShootPose(AllianceColor alliance) {
        Pose2d base = new Pose2d(BACK_SHOOT_X, BACK_SHOOT_Y, 0.0);
        Pose2d pose = alliance == AllianceColor.RED ? base : mirrorPoseForBlue(base);
        Vector2d basket = getActiveBasketPosition();
        double heading = getAimHeadingFromRobotPose(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()), basket);
        return new Pose2d(pose.getX(), pose.getY(), heading);
    }

    private Pose2d getShootingPoseFromStart(Pose2d startPose) {
        Pose2d base = startPose != null ? startPose : getStartPoseForPlan(allianceColor, AutonomousPlan.SHOOT_THREE_FROM_AUDIENCE);
        double shotX = base.getX() + SHOOTING_X_DELTA_FROM_START_INCHES;
        double shotY = base.getY();
        Vector2d basket = getActiveBasketPosition();
        double heading = getAimHeadingFromRobotPose(new Pose2d(shotX, shotY, base.getHeading()), basket);
        return new Pose2d(shotX, shotY, heading);
    }

    public static Pose2d getStartPoseForPlan(AllianceColor allianceColor, AutonomousPlan plan) {
        switch (plan) {
            case LEAVE_FROM_GOAL:
                return getGoalStartPose(allianceColor);
            case LEAVE_FROM_AUDIENCE:
                return getAudienceStartPose(allianceColor);
            case SHOOT_THREE_FROM_AUDIENCE:
                return getAudienceStartPose(allianceColor);
            case SHOOT_THREE_FROM_GOAL:
                return getGoalStartPose(allianceColor);
            case SHOOT_COLLECT_SHOOT_FROM_AUDIENCE:
                return getAudienceStartPose(allianceColor);
            case SHOOT_COLLECT_SHOOT_FROM_GOAL:
                return getGoalStartPose(allianceColor);
            case DRIVE_SQUARE_TEST:
                return getAudienceStartPose(allianceColor);
            default:
                return getGoalStartPose(allianceColor);
        }
    }

    private static Pose2d poseFromHeadingAndOffset(Vector2d center, double headingRadians, double forwardOffsetInches) {
        double dx = Math.cos(headingRadians) * forwardOffsetInches;
        double dy = Math.sin(headingRadians) * forwardOffsetInches;
        return new Pose2d(center.getX() + dx, center.getY() + dy, headingRadians);
    }

    private static Pose2d applyRobotRelativeOffset(Pose2d pose, double forwardInches, double leftInches) {
        double heading = pose.getHeading();
        double dx = (forwardInches * Math.cos(heading)) - (leftInches * Math.sin(heading));
        double dy = (forwardInches * Math.sin(heading)) + (leftInches * Math.cos(heading));
        return new Pose2d(pose.getX() + dx, pose.getY() + dy, heading);
    }

    private static Pose2d applyForwardOffset(Pose2d pose, double headingRadians, double forwardOffsetInches) {
        double dx = Math.cos(headingRadians) * forwardOffsetInches;
        double dy = Math.sin(headingRadians) * forwardOffsetInches;
        return new Pose2d(pose.getX() + dx, pose.getY() + dy, pose.getHeading());
    }

    private static List<Vector2d> getTriadCentersForAlliance(AllianceColor allianceColor) {
        List<Vector2d> centers = new ArrayList<>();
        double y = allianceColor == AllianceColor.RED ? TRIAD_CENTER_Y_RED : -TRIAD_CENTER_Y_RED;
        for (double x : TRIAD_CENTER_XS_RED) {
            centers.add(new Vector2d(x, y));
        }
        return centers;
    }

    private static double getTriadCollectHeading(AllianceColor allianceColor) {
        // Intake is the robot's "back" (heading +180). To face +Y on red, set robot heading -90.
        // For blue, intake faces -Y, so robot heading +90.
        return allianceColor == AllianceColor.RED ? Math.toRadians(-90.0) : Math.toRadians(90.0);
    }

    private static void appendCollectPass(
            List<OpModeCommand> commands,
            Vector2d center,
            double headingRadians,
            double approachOffsetInches,
            double exitOffsetInches,
            double intakePower) {
        Pose2d approachPose = poseFromHeadingAndOffset(center, headingRadians, approachOffsetInches);
        Pose2d exitPose = poseFromHeadingAndOffset(center, headingRadians, -exitOffsetInches);
        commands.add(OpModeCommand.intakePowerCommand(intakePower));
        commands.add(OpModeCommand.driveDirectToPoseCommand(approachPose));
        commands.add(OpModeCommand.driveDirectToPoseCommand(exitPose));
        commands.add(OpModeCommand.intakePowerHoldCommand(intakePower, TRIAD_INTAKE_FLUSH_MILLIS));
    }

    private static Vector2d getMostPositiveXTriadCenter(AllianceColor allianceColor) {
        List<Vector2d> triadCenters = getTriadCentersForAlliance(allianceColor);
        if (triadCenters.isEmpty()) {
            return null;
        }
        Vector2d best = triadCenters.get(0);
        for (Vector2d center : triadCenters) {
            if (center.getX() > best.getX()) {
                best = center;
            }
        }
        return best;
    }

    private void appendTriadLineCollectPassForCenter(
            List<OpModeCommand> commands,
            AllianceColor allianceColor,
            double approachOffsetInches,
            double exitOffsetInches,
            Vector2d center) {
        if (center == null) {
            return;
        }
        double heading = getTriadCollectHeading(allianceColor);
        // Red: start lower Y and move toward +Y. Blue: start higher Y and move toward -Y.
        double approachSign = allianceColor == AllianceColor.RED ? -1.0 : 1.0;
        Pose2d approachPose = new Pose2d(center.getX(), center.getY() + (approachSign * approachOffsetInches), heading);
        Pose2d exitPose = new Pose2d(center.getX(), center.getY() - (approachSign * exitOffsetInches), heading);
        approachPose = applyForwardOffset(approachPose, heading, TRIAD_COLLECT_FORWARD_OFFSET_INCHES);
        exitPose = applyForwardOffset(exitPose, heading, TRIAD_COLLECT_FORWARD_OFFSET_INCHES);
        commands.add(OpModeCommand.intakePowerCommand(INTAKE_MOTOR_POWER));
        commands.add(OpModeCommand.driveDirectToPoseCommand(approachPose));
        commands.add(OpModeCommand.driveDirectToPoseScaledCommand(exitPose, TRIAD_COLLECT_DRIVE_POWER_SCALE));
        commands.add(OpModeCommand.intakePowerHoldCommand(INTAKE_MOTOR_POWER, TRIAD_INTAKE_FLUSH_MILLIS));
    }

    private void appendTriadLineCollectPasses(
            List<OpModeCommand> commands,
            AllianceColor allianceColor,
            double approachOffsetInches,
            double exitOffsetInches) {
        List<Vector2d> triadCenters = getTriadCentersForAlliance(allianceColor);
        if (triadCenters.isEmpty()) {
            return;
        }
        appendTriadLineCollectPasses(commands, allianceColor, approachOffsetInches, exitOffsetInches, triadCenters.size());
    }

    private void appendTriadLineCollectPasses(
            List<OpModeCommand> commands,
            AllianceColor allianceColor,
            double approachOffsetInches,
            double exitOffsetInches,
            int maxTriads) {
        List<Vector2d> triadCenters = getTriadCentersForAlliance(allianceColor);
        if (triadCenters.isEmpty()) {
            return;
        }
        int count = Math.min(Math.max(maxTriads, 0), triadCenters.size());
        if (count <= 0) {
            return;
        }
        double heading = getTriadCollectHeading(allianceColor);
        // Red: start lower Y and move toward +Y. Blue: start higher Y and move toward -Y.
        double approachSign = allianceColor == AllianceColor.RED ? -1.0 : 1.0;
        for (int i = 0; i < count; i++) {
            Vector2d center = triadCenters.get(i);
            Pose2d approachPose = new Pose2d(center.getX(), center.getY() + (approachSign * approachOffsetInches), heading);
            Pose2d exitPose = new Pose2d(center.getX(), center.getY() - (approachSign * exitOffsetInches), heading);
            approachPose = applyForwardOffset(approachPose, heading, TRIAD_COLLECT_FORWARD_OFFSET_INCHES);
            exitPose = applyForwardOffset(exitPose, heading, TRIAD_COLLECT_FORWARD_OFFSET_INCHES);
            commands.add(OpModeCommand.intakePowerCommand(INTAKE_MOTOR_POWER));
            commands.add(OpModeCommand.driveDirectToPoseCommand(approachPose));
            commands.add(OpModeCommand.driveDirectToPoseScaledCommand(exitPose, TRIAD_COLLECT_DRIVE_POWER_SCALE));
            commands.add(OpModeCommand.intakePowerHoldCommand(INTAKE_MOTOR_POWER, TRIAD_INTAKE_FLUSH_MILLIS));
        }
    }

    private void appendTriadCollectPasses(
            List<OpModeCommand> commands,
            List<Vector2d> triadCenters,
            double approachOffsetInches,
            double exitOffsetInches) {
        if (triadCenters == null || triadCenters.isEmpty()) {
            return;
        }
        for (Vector2d center : triadCenters) {
            appendCollectPass(
                    commands,
                    center,
                    AUTO_COLLECT_HEADING_RADIANS,
                    approachOffsetInches,
                    exitOffsetInches,
                    INTAKE_MOTOR_POWER);
        }
    }

    private List<OpModeCommand> buildAutonomousCommands(AutonomousPlan plan) {
        List<OpModeCommand> commands = new ArrayList<>();
        switch (plan) {
            case SHOOT_THREE_FROM_AUDIENCE: {
                Pose2d start = autonomousStartPose != null ? autonomousStartPose : getStartPoseForPlan(allianceColor, plan);
                Pose2d shootingPose = getShootingPoseFromStart(start);
                commands.add(OpModeCommand.shooterEnableCommand(true));
                commands.add(OpModeCommand.waitCommand(750.0));
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsWithExpectedShotsCommand(
                        TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS, TOROID_KNOWN_OPENING_SHOT_COUNT));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.shooterEnableCommand(false));
                commands.add(OpModeCommand.driveDirectToPoseCommand(getLeaveTargetPose(allianceColor)));
                break;
            }
            case SHOOT_THREE_FROM_GOAL: {
                Pose2d shootingPose = getBackShootPose(allianceColor);
                commands.add(OpModeCommand.shooterEnableCommand(true));
                commands.add(OpModeCommand.waitCommand(750.0));
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsWithExpectedShotsCommand(
                        TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS, TOROID_KNOWN_OPENING_SHOT_COUNT));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.shooterEnableCommand(false));
                commands.add(OpModeCommand.driveDirectToPoseCommand(getGoalStartLeavePose(allianceColor)));
                break;
            }
            case SHOOT_COLLECT_SHOOT_FROM_AUDIENCE: {
                Pose2d start = autonomousStartPose != null ? autonomousStartPose : getStartPoseForPlan(allianceColor, plan);
                Pose2d shootingPose = getShootingPoseFromStart(start);
                commands.add(OpModeCommand.shooterEnableCommand(true));
                commands.add(OpModeCommand.waitCommand(750.0));
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsWithExpectedShotsCommand(
                        TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS, TOROID_KNOWN_OPENING_SHOT_COUNT));
                commands.add(OpModeCommand.waitCommand(300.0));
                appendTriadLineCollectPassForCenter(
                        commands,
                        allianceColor,
                        TRIAD_APPROACH_Y_OFFSET,
                        TRIAD_EXIT_Y_OFFSET,
                        getMostPositiveXTriadCenter(allianceColor));
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsCommand(TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.shooterEnableCommand(false));
                commands.add(OpModeCommand.driveDirectToPoseCommand(getLeaveTargetPose(allianceColor)));
                break;
            }
            case SHOOT_COLLECT_SHOOT_FROM_GOAL: {
                Pose2d shootingPose = getBackShootPose(allianceColor);
                commands.add(OpModeCommand.shooterEnableCommand(true));
                commands.add(OpModeCommand.waitCommand(750.0));
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsWithExpectedShotsCommand(
                        TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS, TOROID_KNOWN_OPENING_SHOT_COUNT));
                commands.add(OpModeCommand.waitCommand(300.0));
                appendTriadLineCollectPasses(commands, allianceColor, TRIAD_APPROACH_Y_OFFSET, TRIAD_EXIT_Y_OFFSET, 1);
                commands.add(OpModeCommand.driveDirectToPosePreciseCommand(shootingPose));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.toroidShootRevolutionsCommand(TOROID_SHOOT_REVOLUTIONS, TOROID_SHOOT_TIMEOUT_MILLIS));
                commands.add(OpModeCommand.waitCommand(300.0));
                commands.add(OpModeCommand.shooterEnableCommand(false));
                commands.add(OpModeCommand.driveDirectToPoseCommand(getGoalStartLeavePose(allianceColor)));
                break;
            }
            case DRIVE_SQUARE_TEST: {
                Pose2d start = autonomousStartPose != null ? autonomousStartPose : getStartPoseForPlan(allianceColor, plan);
                Pose2d forward = applyRobotRelativeOffset(start, 12.0, 0.0);
                commands.add(OpModeCommand.driveDirectToPoseCommand(forward));
                break;
            }
            case LEAVE_FROM_AUDIENCE:
            case LEAVE_FROM_GOAL:
            default:
                // Single-move leave: target depends on alliance.
                if (plan == AutonomousPlan.LEAVE_FROM_GOAL) {
                    commands.add(OpModeCommand.driveDirectToPoseCommand(getGoalStartLeavePose(allianceColor)));
                } else {
                    commands.add(OpModeCommand.driveDirectToPoseCommand(getLeaveTargetPose(allianceColor)));
                }
                break;
        }
        return commands;
    }

    private void setCommandSequence(List<OpModeCommand> commands) {
        setCommandSequence(OpModeState.STOPPED_UNTIL_END, commands);
    }

    private void setCommandSequence(OpModeState _continuationState, List<OpModeCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        currentCommand = null;
        currentCommandActionsStarted = false;
        continuationState = _continuationState;
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    public void shutDown() {
        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrivePower(new Pose2d());
        limelight.stop();
        if (visionPortal != null) {
            visionPortal.close();
        }
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

        double combined = Math.hypot(transMag, Math.abs(h));
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
