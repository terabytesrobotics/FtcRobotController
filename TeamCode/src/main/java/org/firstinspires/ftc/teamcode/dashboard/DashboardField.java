package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;

/** Shared FTC Dashboard rendering kept outside the mode-independent Robot class. */
public final class DashboardField {
    private static final String BIOBUZZ_FIELD_IMAGE = "/images/biobuzz-field-v1.png";
    private static final double FIELD_SIZE_INCHES = 144.0;

    private DashboardField() {
    }

    public static void drawBackground(Canvas field) {
        // Dashboard image coordinates use the page frame: (0, 0) is the upper-left corner.
        field.drawImage(
                BIOBUZZ_FIELD_IMAGE,
                0.0, 0.0,
                FIELD_SIZE_INCHES, FIELD_SIZE_INCHES);
    }

    public static void drawPose(
            Canvas field,
            double xInches,
            double yInches,
            double headingRadians,
            double radiusInches,
            String color) {
        field.setStroke(color);
        field.strokeCircle(xInches, yInches, radiusInches);
        field.strokeLine(
                xInches,
                yInches,
                xInches + Math.cos(headingRadians) * radiusInches,
                yInches + Math.sin(headingRadians) * radiusInches);
    }
}
