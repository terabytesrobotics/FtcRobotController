package org.firstinspires.ftc.teamcode.element;

import org.firstinspires.ftc.teamcode.util.data.Rect;
import org.firstinspires.ftc.teamcode.util.data.StaticGameElementType;

public abstract class BaseStaticGameElement {
    public abstract String getID();
    public abstract StaticGameElementType getType();
    public abstract Rect getBounds();
}
