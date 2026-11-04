package org.firstinspires.ftc.teamcode.element;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.data.Rect;
import org.firstinspires.ftc.teamcode.util.data.StaticGameElementType;

public class FlowerGameElement extends BaseStaticGameElement {
    private Rect boundingBox;
    private String id;

    // 6 inches with a .5in buffer and converted to MM
    private double edgeLength = 6.5 * Constants.IN_TO_MM;

    public FlowerGameElement(double centerXIn, double centerYIn, String id) {
        this.id = id;
        boundingBox = new Rect(centerXIn * Constants.IN_TO_MM - edgeLength / 2,
                centerYIn * Constants.IN_TO_MM - edgeLength / 2, edgeLength, edgeLength);
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
