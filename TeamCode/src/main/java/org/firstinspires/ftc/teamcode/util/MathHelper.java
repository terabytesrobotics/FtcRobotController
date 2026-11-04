package org.firstinspires.ftc.teamcode.util;

public class MathHelper {
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }

    public static double approxDistanceTo(double x1, double y1, double x2, double y2) {
        double dx = Math.abs(x1 - x2);
        double dy = Math.abs(y1 - y2);
        double max = Math.max(dx, dy);
        double min = Math.min(dx, dy);
        return max + 0.4142 * min;
    }
}
