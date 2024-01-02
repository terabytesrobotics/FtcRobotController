package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum AlliancePose {

    BACKSTAGE_START(
            new Pose2d(12, 64.25, Math.toRadians(90)),
            new Pose2d(12, -64.25, Math.toRadians(270))),
    FRONTSTAGE_START(
            new Pose2d(-36, 64.25, Math.toRadians(90)),
            new Pose2d(-36, -64.25, Math.toRadians(270)));

    public Pose2d BluePose;
    public Pose2d RedPose;

    AlliancePose(Pose2d bluePose, Pose2d redPose) {
        this.BluePose = bluePose;
        this.RedPose = redPose;
    }

    public Vector2d getPixelTargetPosition(AllianceColor allianceColor, AlliancePropPosition propPosition) {
        switch (this) {
            case BACKSTAGE_START:
                switch (allianceColor) {
                    case RED:
                        return propPosition.RedBackstagePixelTarget;
                    case BLUE:
                        return propPosition.BlueBackstagePixelTarget;
                    default:
                        return new Vector2d();
                }
            case FRONTSTAGE_START:
                switch (allianceColor) {
                    case RED:
                        return propPosition.RedFrontstagePixelTarget;
                    case BLUE:
                        return propPosition.BlueFrontstagePixelTarget;
                    default:
                        return new Vector2d();
                }
            default:
                return new Vector2d();
        }
    }
}
