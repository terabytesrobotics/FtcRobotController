package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum NibusApproach {
    RED_SCORING_APPROACH(new Pose2d(36, 36, 0), 0),
    BLUE_SCORING_APPROACH(new Pose2d(36, -36, 0), 0),
    RED_COLLECT_APPROACH(
            new Pose2d(
                    CenterStageConstants.RED_COLLECT.getX(),
                    CenterStageConstants.RED_COLLECT.getY(),
                    Math.toRadians(135)),
                Math.toRadians(40)
            ),
    BLUE_COLLECT_APPROACH(
            new Pose2d(
                    CenterStageConstants.BLUE_COLLECT.getX(),
                    CenterStageConstants.BLUE_COLLECT.getY(),
                    Math.toRadians(225)),
            Math.toRadians(40)),
    RED_COLLECT_APPROACH_1(
            new Pose2d(
                    CenterStageConstants.RED_COLLECT_1.getX(),
                    CenterStageConstants.RED_COLLECT_1.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80)),
    RED_COLLECT_APPROACH_2(
            new Pose2d(
                    CenterStageConstants.RED_COLLECT_2.getX(),
                    CenterStageConstants.RED_COLLECT_2.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80)),
    RED_COLLECT_APPROACH_3(
            new Pose2d(
                    CenterStageConstants.RED_COLLECT_3.getX(),
                    CenterStageConstants.RED_COLLECT_3.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80)),
    BLUE_COLLECT_APPROACH_1(
            new Pose2d(
                    CenterStageConstants.BLUE_COLLECT_1.getX(),
                    CenterStageConstants.BLUE_COLLECT_1.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80)),
    BLUE_COLLECT_APPROACH_2(
            new Pose2d(
                    CenterStageConstants.BLUE_COLLECT_2.getX(),
                    CenterStageConstants.BLUE_COLLECT_2.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80)),
    BLUE_COLLECT_APPROACH_3(
            new Pose2d(
                    CenterStageConstants.BLUE_COLLECT_3.getX(),
                    CenterStageConstants.BLUE_COLLECT_3.getY(),
                    Math.toRadians(180)),
            Math.toRadians(80));

    public final Pose2d Pose;
    public final double ApproachHeadingRange;

    NibusApproach(Pose2d pose, double approachHeadingRange) {
        Pose = pose;
        ApproachHeadingRange = approachHeadingRange;
    }

    public NibusApproach nextPose() {
        return NibusApproach.values()[(ordinal() + 1) % NibusApproach.values().length];
    }
}
