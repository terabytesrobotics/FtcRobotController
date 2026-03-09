package org.firstinspires.ftc.teamcode;

public class Constants {

    static final double TURN_ERROR_THRESHOLD = Math.PI / 24;
    static final double SPEED_GAIN = (1d / 36) * 1.15;
    static final double TURN_GAIN =  1d / (5 * Math.PI / 8);
    public static final double FRONT_CAMERA_OFFSET_INCHES = 7.5;
    public static final double FRONT_CAMERA_LATERAL_OFFSET_INCHES = 0.0;
    // Height of the camera above the field plane (robot origin assumed at carpet height). Tune on-robot.
    public static final double FRONT_CAMERA_HEIGHT_INCHES = 15.5;
    public static final double DRIVE_TO_POSE_THRESHOLD = 1.5f;
    public static double APRIL_TAG_RECOGNITION_MAX_RANGE = 144;
    public static double APRIL_TAG_RECOGNITION_MIN_RANGE = 5;
    public static double APRIL_TAG_RECOGNITION_YAW_THRESHOLD = Math.PI / 6;
    public static double APRIL_TAG_RECOGNITION_BEARING_THRESHOLD = Math.PI / 6;
    // Sanity limits for pose acceptance (looser than recognition thresholds).
    public static double APRIL_TAG_SANITY_MAX_RANGE = 183.3; // ~15.3 feet
    public static double APRIL_TAG_SANITY_MAX_BEARING = Math.toRadians(60.0);
    public static double APRIL_TAG_SANITY_MAX_YAW = Math.toRadians(60.0);
    // Only trust tag-based pose corrections when we're close and squared up.
    public static double APRIL_TAG_TRUSTED_MAX_RANGE = 96; // inches
    public static double APRIL_TAG_TRUSTED_MAX_BEARING = Math.toRadians(12.0);
    public static double APRIL_TAG_TRUSTED_MAX_YAW = Math.toRadians(10.0);
    // Calibration knobs for camera pose estimates.
    // If tag range feels consistently long/short, adjust APRIL_TAG_RANGE_SCALE (e.g., 0.93 to shrink).
    public static double APRIL_TAG_RANGE_SCALE = 0.825;
    // If camera is pitched up/down relative to robot forward, compensate here (degrees).
    public static double APRIL_TAG_ELEVATION_OFFSET_DEG = 0.0;
    public static double APRIL_TAG_MIN_DECISION_MARGIN = 60.0;
    public static final int APRIL_TAG_QUEUE_CAPACITY = 8;
    // Minimum samples needed before fusing (useful for smoothing but still lets us react quickly).
    public static final int APRIL_TAG_MIN_QUEUE_SAMPLES = 2;
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
    public static final double TOROID_JOYSTICK_DEADBAND = 0.15;

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
