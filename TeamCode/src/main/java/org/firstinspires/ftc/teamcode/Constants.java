package org.firstinspires.ftc.teamcode;

public class Constants {

    static final double TURN_ERROR_THRESHOLD = Math.PI / 24;
    static final double SPEED_GAIN = 1d / 36;
    static final double TURN_GAIN =  1d / (5 * Math.PI / 8);
    public static final int FRONT_CAMERA_OFFSET_INCHES = 8;
    public static final double FRONT_CAMERA_LATERAL_OFFSET_INCHES = 0.5;
    // Height of the camera above the field plane (robot origin assumed at carpet height). Tune on-robot.
    public static final double FRONT_CAMERA_HEIGHT_INCHES = 14.0;
    public static final double DRIVE_TO_POSE_THRESHOLD = 1.5f;
    public static double APRIL_TAG_RECOGNITION_MAX_RANGE = 144;
    public static double APRIL_TAG_RECOGNITION_MIN_RANGE = 5;
    public static double APRIL_TAG_RECOGNITION_YAW_THRESHOLD = Math.PI / 6;
    public static double APRIL_TAG_RECOGNITION_BEARING_THRESHOLD = Math.PI / 6;
    public static final int APRIL_TAG_QUEUE_CAPACITY = 8;
    // Minimum samples needed before fusing (useful for smoothing but still lets us react quickly).
    public static final int APRIL_TAG_MIN_QUEUE_SAMPLES = 1;
    public static double APRIL_TAG_VARIANCE_TRANSLATION_THRESHOLD = 2.0;
    public static double APRIL_TAG_VARIANCE_HEADING_THRESHOLD = Math.PI / 16.0;
    // How aggressively to blend AprilTag measurements toward the odometry estimate.
    public static double APRIL_TAG_BLEND_TRANSLATION_WEIGHT = 0.35;
    public static double APRIL_TAG_BLEND_HEADING_WEIGHT = 0.25;
    // Snap to the tag outright if odometry drifts beyond these limits.
    public static double APRIL_TAG_MAX_CORRECTION_DISTANCE = 18.0;
    public static double APRIL_TAG_MAX_CORRECTION_HEADING = Math.toRadians(25.0);
    public static final double SLOW_MODE_SCALE = 0.3;
    public static final double FAST_MODE_SCALE = 1;
    public static final boolean INVERT_TILT_SERVO = true; // Switches direction for new servo Set to false if switching back

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
