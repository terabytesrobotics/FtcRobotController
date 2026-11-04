package org.firstinspires.ftc.teamcode.util.data;

import androidx.annotation.Nullable;

public class Node {
    public double x;
    public double y;
    public double g;
    public double h;
    public Node parent;

    public Node(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getF() {
        return g + h;
    }

    @Override
    public boolean equals(@Nullable Object other) {
        return other instanceof Node && ((Node) other).x == x && ((Node) other).y == y;
    }
}
