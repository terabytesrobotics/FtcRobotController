package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.Map;

public enum CenterStageAprilTags {
    BLUE_BACKDROP_LEFT(1, new Pose2d(64, 41, Math.toRadians(180))),
    BLUE_BACKDROP_CENTER(2, new Pose2d(64, 35, Math.toRadians(180))),
    BLUE_BACKDROP_RIGHT(3, new Pose2d(64, 29, Math.toRadians(180))),
    RED_BACKDROP_LEFT(4, new Pose2d(64, -29, Math.toRadians(180))),
    RED_BACKDROP_CENTER(5, new Pose2d(64, -35, Math.toRadians(180))),
    RED_BACKDROP_RIGHT(6, new Pose2d(64, -41, Math.toRadians(180))),
    RED_WALL_LARGE(7, new Pose2d(
            -72,
            -40.5,
            0)),
    RED_WALL_SMALL(8, new Pose2d(
            -72,
            -35,
            0)),
    BLUE_WALL_SMALL(9, new Pose2d(
            -72,
            35,
            0)),
    BLUE_WALL_LARGE(10, new Pose2d(
            -72,
            40.5,
            0));


    public int Id;
    public Pose2d Pose;

    CenterStageAprilTags(int id, Pose2d pose) {
        Id = id;
        Pose = pose;
    }

    public static CenterStageAprilTags getTag(int id) {
        for (CenterStageAprilTags tag: values()) {
            if (tag.Id == id) {
                return tag;
            }
        }
        return null;
    }

    public Pose2d facingPose() {
        return new Pose2d(Pose.getX(), Pose.getY(), Angle.norm(Pose.getHeading() + Math.PI));
    }
}
