package org.firstinspires.ftc.teamcode.element;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.data.Rect;
import org.firstinspires.ftc.teamcode.util.data.StaticGameElementType;

public class HiveLegGameElement extends BaseStaticGameElement {
    private Rect boundingBox;
    private String id;

    private double longLength = 48 * Constants.IN_TO_MM;
    private double width = 4 * Constants.IN_TO_MM;

    public HiveLegGameElement(double centerXIn, double centerYIn, String id) {
        this.id = id;
        boundingBox = new Rect(centerXIn * Constants.IN_TO_MM - width / 2,
                centerYIn * Constants.IN_TO_MM - longLength / 2, width, longLength);
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public StaticGameElementType getType() {
        return StaticGameElementType.FLOWER;
    }

    @Override
    public Rect getBounds() {
        return boundingBox;
    }
}
