package org.firstinspires.ftc.teamcode;

public class NibusConstants {

    public static final int ARM_TOLERANCE = 10;
    public static final double GEAR_RATIO = 13.7d;
    public static final double WORM_RATIO = 28.0d;
    public static final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;
    public static double ARM_CONTROL_P = 0.005, ARM_CONTROL_I = 0.005, ARM_CONTROL_D = 0.0002;
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

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
