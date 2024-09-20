package org.firstinspires.ftc.teamcode;

public class TerabytesIntoTheDeepConstants {

    static final double TURN_ERROR_THRESHOLD = Math.PI / 24;
    static final double SPEED_GAIN  =  0.125;
    static final double TURN_GAIN =  .0265 / (Math.PI / 180);
    static final double MAX_AUTO_SPEED = 0.75;
    static final double MAX_AUTO_STRAFE = 0.75;
    static final double MAX_AUTO_TURN  = 1;
    public static final int FRONT_CAMERA_OFFSET_INCHES = 7;
    public static final int BACK_CAMERA_OFFSET_INCHES = 0;
    public static final double DRIVE_TO_POSE_THRESHOLD = 1.5f;
    public static final int POSITION_ACQUIRED_INDICATE_MILLIS = 1000;
    public static final int POSITION_ACQUIRED_PULSE_MILLIS = 100;
    public static double APRIL_TAG_RECOGNITION_MAX_RANGE = 32;
    public static double APRIL_TAG_RECOGNITION_MIN_RANGE = 5;
    public static double APRIL_TAG_RECOGNITION_YAW_THRESHOLD = Math.PI / 6;
    public static double APRIL_TAG_RECOGNITION_BEARING_THRESHOLD = Math.PI / 6;
    public static final int APRIL_TAG_QUEUE_CAPACITY = 8;
    public static final double SLOW_MODE_SCALE = 0.3;
    public static final double FAST_MODE_SCALE = 1;

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
