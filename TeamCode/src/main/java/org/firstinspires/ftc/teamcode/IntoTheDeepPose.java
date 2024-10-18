package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public enum IntoTheDeepPose {
    AUTON_START_MID(IntoTheDeepFieldPosition.AUTON_START_MID, Math.PI / 2.0),
    AUTON_PRE_NEUTRAL_PUSH_STAGING(IntoTheDeepFieldPosition.AUTON_PRE_NEUTRAL_PUSH_STAGING, Math.PI / 2.0),
    AUTON_PRE_NEUTRAL_PUSH_1(IntoTheDeepFieldPosition.AUTON_PRE_NEUTRAL_PUSH_1, Math.PI / 2.0),
    AUTON_PRE_NEUTRAL_PUSH_2(IntoTheDeepFieldPosition.AUTON_PRE_NEUTRAL_PUSH_2, Math.PI / 2.0),
    AUTON_NEUTRAL_PUSH_TARGET_1(IntoTheDeepFieldPosition.AUTON_NEUTRAL_PUSH_TARGET, Math.toRadians(90 - 18.43)),
    AUTON_NEUTRAL_PUSH_TARGET_2(IntoTheDeepFieldPosition.AUTON_NEUTRAL_PUSH_TARGET, Math.toRadians(90)),
    AUTON_PARK_TARGET(IntoTheDeepFieldPosition.AUTON_PARK_TARGET, Math.PI / 2.0);

    public final Pose2d BluePose;
    public final Pose2d RedPose;

    IntoTheDeepPose(IntoTheDeepFieldPosition fieldPosition, double headingBlue) {
        BluePose = new Pose2d(fieldPosition.BluePosition, headingBlue);
        RedPose = new Pose2d(fieldPosition.RedPosition, headingBlue + Math.PI);
    }

    public Pose2d getPose(AllianceColor allianceColor) {
        switch (allianceColor) {
            case RED:
                return RedPose;
            case BLUE:
                return BluePose;
            default:
                return new Pose2d();
        }
    }
}
