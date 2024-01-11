package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum PoseOfInterest {
    RED_SCORING_APPROACH(new Pose2d(36, 36, 0)),
    RANDOM_POINT(new Pose2d(36, 0, 0)),
    BLUE_SCORING_APPROACH(new Pose2d(36, -36, 0)),
    RED_COLLECT_APPROACH(
            new Pose2d(
                    CenterStageConstants.RED_COLLECT.getX(),
                    CenterStageConstants.RED_COLLECT.getY(),
                    Math.toRadians(135))),
    BLUE_COLLECT_APPROACH(
            new Pose2d(
                    CenterStageConstants.BLUE_COLLECT.getX(),
                    CenterStageConstants.BLUE_COLLECT.getY(),
                    Math.toRadians(225)));
    ;
    public final Pose2d Pose;

    PoseOfInterest(Pose2d pose) {
        this.Pose = pose;
    }

    public PoseOfInterest nextPose() {
        return PoseOfInterest.values()[(ordinal() + 1) % PoseOfInterest.values().length];
    }
}
