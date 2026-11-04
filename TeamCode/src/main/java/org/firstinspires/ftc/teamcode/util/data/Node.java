package org.firstinspires.ftc.teamcode.util.data;

import androidx.annotation.Nullable;

public class Node {
    public int x;
    public int y;
    public int g;
    public int h;
    public Node parent;

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public int getF() {
        return g + h;
    }

    @Override
    public boolean equals(@Nullable Object other) {
        return other instanceof Node && ((Node) other).x == x && ((Node) other).y == y;
    }
}
