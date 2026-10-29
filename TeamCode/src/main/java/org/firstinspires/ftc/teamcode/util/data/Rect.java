package org.firstinspires.ftc.teamcode.util.data;

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
}
