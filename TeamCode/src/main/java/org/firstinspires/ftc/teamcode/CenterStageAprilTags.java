package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum CenterStageAprilTags {
    BLUE_BACKDROP_LEFT(1, new Pose2d(64, 41, Math.toRadians(180))),
    BLUE_BACKDROP_CENTER(2, new Pose2d(64, 35, Math.toRadians(180))),
    BLUE_BACKDROP_RIGHT(3, new Pose2d(64, 29, Math.toRadians(180))),
    RED_BACKDROP_LEFT(4, new Pose2d(64, -29, Math.toRadians(180))),
    RED_BACKDROP_CENTER(5, new Pose2d(64, -35, Math.toRadians(180))),
    RED_BACKDROP_RIGHT(6, new Pose2d(64, -41, Math.toRadians(180))),
    RED_WALL_SMALL(7, new Pose2d(
            CenterStageConstants.COLLECT_1.getX(),
            CenterStageConstants.COLLECT_1.getY(),
            0)),
    RED_WALL_LARGE(8, new Pose2d(
            CenterStageConstants.COLLECT_1.getX(),
            CenterStageConstants.COLLECT_1.getY() - 5,
            0)),
    BLUE_WALL_SMALL(9, new Pose2d(
            CenterStageConstants.COLLECT_6.getX(),
            CenterStageConstants.COLLECT_6.getY(),
            0)),
    BLUE_WALL_LARGE(10, new Pose2d(
            CenterStageConstants.COLLECT_6.getX(),
            CenterStageConstants.COLLECT_6.getY() + 5,
            0));

    public int Id;
    public Pose2d Pose;
    CenterStageAprilTags(int id, Pose2d pose) {
        Id = id;
        Pose = pose;
    }
}
