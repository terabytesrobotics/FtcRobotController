package org.firstinspires.ftc.teamcode.element;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.data.Node;
import org.firstinspires.ftc.teamcode.util.data.Rect;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StaticGameElementMappings {
    public static List<BaseStaticGameElement> elements = List.of(
            new FlowerGameElement(-Constants.MAX_FIELD_IN + 48, Constants.MAX_FIELD_IN - 3, "flower_blue_up"),
            new FlowerGameElement(-Constants.MAX_FIELD_IN + 3, -Constants.MAX_FIELD_IN + 48, "flower_blue_down"),
            new FlowerGameElement(Constants.MAX_FIELD_IN - 48, -Constants.MAX_FIELD_IN + 3, "flower_red_up"),
            new FlowerGameElement(Constants.MAX_FIELD_IN - 3, Constants.MAX_FIELD_IN - 48, "flower_red_down"),
            new HiveLegGameElement(-24, 0, "hive_leg_blue"),
            new HiveLegGameElement(24, 0, "hive_leg_red")
    );

    // biobuzz static elements are all axis aligned
    public static Set<Node> obstacleNodes(double cellSamplingMM) {
        Set<Node> nodes = new HashSet<>();

        for (BaseStaticGameElement obstacle : elements) {
            Rect bounds = obstacle.getBounds();

            for (double x = bounds.topLeft.x;
                 x < bounds.topRight.x;
                 x += cellSamplingMM) {
                for (double y = bounds.topLeft.y;
                     y < bounds.bottomLeft.y;
                     y += cellSamplingMM) {

                    nodes.add(new Node(x, y));
                }
            }
        }

        return nodes;
    }
}
