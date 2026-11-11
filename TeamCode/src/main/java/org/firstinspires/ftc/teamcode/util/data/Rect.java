package org.firstinspires.ftc.teamcode.util.data;

import android.annotation.SuppressLint;

public class Rect {
    public Rect(double x, double y, double width, double height) {
        this.topLeft = new TwoDimPos(x, y);
        this.topRight = new TwoDimPos(x + width, y);
        this.bottomRight = new TwoDimPos(x + width, y + height);
        this.bottomLeft = new TwoDimPos(x, y + height);
    }

    // clockwise points starting at top left
    public Rect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        this.topLeft = new TwoDimPos(x1, y1);
        this.topRight = new TwoDimPos(x2, y2);
        this.bottomRight = new TwoDimPos(x3, y3);
        this.bottomLeft = new TwoDimPos(x4, y4);
    }

    public TwoDimPos topLeft;
    public TwoDimPos topRight;
    public TwoDimPos bottomLeft;
    public TwoDimPos bottomRight;

    @SuppressLint("DefaultLocale")
    public String asString() {
        double width = Math.hypot(topRight.x - topLeft.x, topRight.y - topLeft.y);
        double height = Math.hypot(bottomLeft.x - topLeft.x, bottomLeft.y - topLeft.y);

        return String.format(
                "Rect {\n" +
                        "  Width: %.2f, Height: %.2f\n" +
                        "  Top Left:     (%.2f, %.2f)\n" +
                        "  Top Right:    (%.2f, %.2f)\n" +
                        "  Bottom Left:  (%.2f, %.2f)\n" +
                        "  Bottom Right: (%.2f, %.2f)\n" +
                        "}",
                width, height,
                topLeft.x, topLeft.y,
                topRight.x, topRight.y,
                bottomLeft.x, bottomLeft.y,
                bottomRight.x, bottomRight.y
        );
    }
}
