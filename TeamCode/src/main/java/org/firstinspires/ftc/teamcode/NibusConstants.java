package org.firstinspires.ftc.teamcode;

public class NibusConstants {

    public static final int ARM_TOLERANCE = 10;
    public static final double GEAR_RATIO = 13.7d;
    public static final double WORM_RATIO = 28.0d;
    public static final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;
    public static double ARM_CONTROL_P = 0.0075, ARM_CONTROL_I = 0.000, ARM_CONTROL_D = 0.0000;
    // Angle below horizontal at start in degrees.  horizontal is 0.
    public static final double ARM_DEGREE_OFFSET_FROM_HORIZONTAL = -37d;
    public static final double ARM_MAX_ANGLE = 180d;
    public static final double EXTENDER_MAX_LENGTH_INCHES = 7.48d;
    public static final double EXTENDER_POWER = 0.8d;
    public static final double EXTENDER_GEAR_RATIO = 5.2d;
    public static final double EXTENDER_TICS_PER_INCH = (EXTENDER_GEAR_RATIO * 28 / 0.8) * 2.54;
    public static final int PROP_CAMERA_WIDTH_PIXELS = 640;
    public static final int PROP_CAMERA_HEIGHT_PIXELS = 480;
    public static final int PROP_CAMERA_ROW_COUNT = 3;
    public static final int PROP_CAMERA_COLUMN_COUNT = 3;
    public static final double ARM_DEGREE_TRIM_INCREMENT = 1;
    public static final int ARM_MAX_TRIM_INCREMENTS = 100;
    public static final double WRIST_SERVO_TRIM_INCREMENT = 0.02;
    public static final int WRIST_MAX_TRIM_INCREMENTS = 100;
    public static final int END_GAME_BEGINS_MILLIS = 2 * 60 * 1000;
    public static final double LAUNCH_WRIST_POSITION = 0d;
    public static final int FRAME_DELAY_MILLIS = 100;
    public static final int DELAY_FRAMES = 10;
    public static final int PROCESS_FRAMES = 10;
    static final double DISTANCE_ERROR_THRESHOLD = 0.5;
    static final double STRAFE_ERROR_THRESHOLD = Math.PI / 32;
    static final double TURN_ERROR_THRESHOLD = Math.PI / 24;
    static final double DESIRED_DISTANCE = 8.0;
    static final double SPEED_GAIN  =  0.125;
    static final double STRAFE_GAIN =  .01 / (Math.PI / 180);
    static final double TURN_GAIN =  .0265 / (Math.PI / 180);
    static final double MAX_AUTO_SPEED = 0.68;
    static final double MAX_AUTO_STRAFE = 0.68;
    static final double MAX_AUTO_TURN  = 1;
    static final int APPROACH_SETTLE_TIME_MS = 500;
    public static final int FRONT_CAMERA_OFFSET_INCHES = 7;
    public static final double FRONT_CAMERA_IDEAL_COLLECTION_DISTANCE = 14;

    public static final int BACK_CAMERA_OFFSET_INCHES = 0;
    public static final double COLLECT_HEAD_BASE_OFFSET_X = 14; // This is magnitude, not direction (usually backwards on robot in collect state.)
    public static final double COLLECT_HEAD_BLUE_GRABBER_OFFSET_Y = 2;
    public static final double COLLECT_HEAD_GREEN_GRABBER_OFFSET_Y = -2;
    public static final double COLLECT_HEAD_HEADING_OFFSET = Math.PI;
    public static final double DRIVE_TO_POSE_THRESHOLD = 1.5f;
    public static final int POSITION_ACQUIRED_INDICATE_MILLIS = 1000;
    public static final int POSITION_ACQUIRED_PULSE_MILLIS = 100;
    public static double APRIL_TAG_RECOGNITION_MAX_RANGE = 30;
    public static double APRIL_TAG_RECOGNITION_MIN_RANGE = 6;
    public static double APRIL_TAG_RECOGNITION_YAW_THRESHOLD = Math.PI / 6;
    public static double APRIL_TAG_RECOGNITION_BEARING_THRESHOLD = Math.PI / 6;
    public static int ARM_TICK_TOLERANCE = 20;
    public static int EXTENDER_TICK_TOLERANCE = 20;
    public static final int APRIL_TAG_QUEUE_CAPACITY = 8;
    public static final double SLOW_MODE_SCALE = 0.3;
    public static final double FAST_MODE_SCALE = 1;
    public static double SCORING_HEIGHT_MAX = 24;
    public static double WHITE_WRIST_SERVO_OFFSET_FROM_RED = 0.02;

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
