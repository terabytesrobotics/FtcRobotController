package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Vector;

public class CenterStageConstants {

    public static final int UPSTAGE_FIELD_X = -30;
    public static final int BACKSTAGE_FIELD_X = 30;
    public static final int LANE_1_Y = 60;
    public static final int LANE_2_Y = 36;
    public static final int LANE_3_Y = 12;

    public static final int LANE_4_Y = -LANE_1_Y;

    public static final int LANE_5_Y = -LANE_2_Y;
    public static final int LANE_6_Y = -LANE_3_Y;

    public static final Vector2d BLUE_COLLECT = new Vector2d(-72, 72);
    public static final Vector2d RED_COLLECT = new Vector2d(-72, -72);

    public static final Vector2d COLLECT_1 = new Vector2d(-72, 36);
    public static final Vector2d COLLECT_2 = new Vector2d(-72, 24);
    public static final Vector2d COLLECT_3 = new Vector2d(-72, 12);
    public static final Vector2d COLLECT_4 = new Vector2d(-72, -12);
    public static final Vector2d COLLECT_5 = new Vector2d(-72, -24);
    public static final Vector2d COLLECT_6 = new Vector2d(-72, -36);

    public static final int[] LANES = {LANE_1_Y, LANE_2_Y, LANE_3_Y, LANE_4_Y, LANE_5_Y, LANE_6_Y};

    public static int getClosestLane(double y) {
        int closestLane = LANES[0];
        double minDiff = Math.abs(y - closestLane);

        for (int lane : LANES) {
            double diff = Math.abs(y - lane);
            if (diff < minDiff) {
                minDiff = diff;
                closestLane = lane;
            }
        }

        return closestLane;
    }
}
