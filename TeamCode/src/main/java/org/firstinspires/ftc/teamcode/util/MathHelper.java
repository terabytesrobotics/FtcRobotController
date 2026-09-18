package org.firstinspires.ftc.teamcode.util;

public class MathHelper {
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }
}
