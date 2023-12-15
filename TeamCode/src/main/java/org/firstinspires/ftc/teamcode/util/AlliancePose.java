package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum AlliancePose {

    BACKSTAGE_START(
            new Pose2d(12, 60, Math.toRadians(270)),
            new Pose2d(12, -60, Math.toRadians(270))),
    FRONTSTAGE_START(
            new Pose2d(-36, 60, Math.toRadians(90)),
            new Pose2d(-36, -60, Math.toRadians(90)));

    public Pose2d BluePose;
    public Pose2d RedPose;

    AlliancePose(Pose2d bluePose, Pose2d redPose) {
        this.BluePose = bluePose;
        this.RedPose = redPose;
    }
}
