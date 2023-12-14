package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum AllianceColor {
    BLUE,
    RED;

    public Pose2d getAbsoluteFieldPose(AlliancePose alliancePose) {
        switch (this) {
            case RED:
                return alliancePose.RedPose;
            case BLUE:
                return alliancePose.BluePose;
            default:
                return new Pose2d();
        }
    }

    public int getAprilTagId(AlliancePropPosition alliancePropPosition) {
        switch (this) {
            case RED:
                return alliancePropPosition.RedAprilTagId;
            case BLUE:
                return alliancePropPosition.BlueAprilTagId;
            default:
                return 0;
        }
    }
}
