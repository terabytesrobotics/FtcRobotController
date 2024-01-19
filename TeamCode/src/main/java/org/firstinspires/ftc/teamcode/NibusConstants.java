package org.firstinspires.ftc.teamcode;

public class NibusConstants {

    public static final int ARM_TOLERANCE = 10;
    public static final double GEAR_RATIO = 13.7d;
    public static final double WORM_RATIO = 28.0d;
    public static final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;
    public static double ARM_CONTROL_P = 0.005, ARM_CONTROL_I = 0.000, ARM_CONTROL_D = 0.0000;
    // Angle below horizontal at start in degrees.  horizontal is 0.
    public static final double ARM_DEGREE_OFFSET_FROM_HORIZONTAL = -37d;
    public static final double ARM_MAX_ANGLE = 180d;
    public static final double EXTENDER_MAX_LENGTH = 19d;
    public static final double EXTENDER_POWER = 0.8d;
    public static final double EXTENDER_GEAR_RATIO = 5.2d;
    public static final double EXTENDER_TICS_PER_CM = EXTENDER_GEAR_RATIO * 28 / 0.8;
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
    static final double TURN_ERROR_THRESHOLD = Math.PI / 32;
    static final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    static final double SPEED_GAIN  =  0.1;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double STRAFE_GAIN =  1.5/Math.PI ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double TURN_GAIN   =  1.5/Math.PI  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_TURN  = Math.PI / 4;   //  Clip the turn speed to this max value (adjust for your robot)
    static final int APPROACH_SETTLE_TIME_MS = 750;
    public static final int FRONT_CAMERA_OFFSET_INCHES = 7;
    public static final int BACK_CAMERA_OFFSET_INCHES = 0;
    public static final double COLLECT_HEAD_BASE_OFFSET_X = -14;
    public static final double DRIVE_TO_POSE_THRESHOLD = 1.0f;
    public static final int POSITION_ACQUIRED_INDICATE_MILLIS = 1000;
    public static final int POSITION_ACQUIRED_PULSE_MILLIS = 100;
    public static double APRIL_TAG_RECOGNITION_RANGE_THRESHOLD = 30;
    public static double APRIL_TAG_RECOGNITION_YAW_THRESHOLD = Math.PI / 6;
    public static double APRIL_TAG_RECOGNITION_BEARING_THRESHOLD = Math.PI / 6;
    public static int ARM_TICK_TOLERANCE = 20;
    public static int EXTENDER_TICK_TOLERANCE = 20;
    public static final int APRIL_TAG_QUEUE_CAPACITY = 8;

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
