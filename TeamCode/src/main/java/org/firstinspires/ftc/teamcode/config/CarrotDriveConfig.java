package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/** Dashboard-adjustable configuration. Per-run controller state belongs in the op mode. */
@Config
public final class CarrotDriveConfig {
    public static double maxTranslationPower = 0.75;
    public static double maxRotationPower = 0.60;
    public static double positionToleranceMm = 8.0;
    public static double headingToleranceDeg = 2.0;
    public static double translationRateMmPerSecond = 600.0;
    public static double rotationRateDegPerSecond = 90.0;
    public static double maxCarrotLeadMm = 300.0;

    private CarrotDriveConfig() {
    }
}
