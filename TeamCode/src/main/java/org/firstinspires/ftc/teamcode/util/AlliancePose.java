package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum AlliancePose {

    // Currently the center of the tile...TODO.
    BACKSTAGE_START(
            new Pose2d(84, 12, 90),
            new Pose2d(84, 132, 90)),
    FRONTSTAGE_START(
            new Pose2d(36, 12, 270),
            new Pose2d(36, 132, 270));

    public Pose2d BluePose;
    public Pose2d RedPose;

    AlliancePose(Pose2d bluePose, Pose2d redPose) {
        this.BluePose = bluePose;
        this.RedPose = redPose;
    }
}
