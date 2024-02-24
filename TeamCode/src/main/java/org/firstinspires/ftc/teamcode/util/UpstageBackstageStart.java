package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum UpstageBackstageStart {
    BACKSTAGE_START(
            new Pose2d(12, 61.5, Math.toRadians(90)),
            new Pose2d(12, -61.5, Math.toRadians(270))),
    FRONTSTAGE_START(
            new Pose2d(-35, 61.5, Math.toRadians(90)),
            new Pose2d(-35, -61.5, Math.toRadians(270)));

    public Pose2d BluePose;
    public Pose2d RedPose;

    UpstageBackstageStart(Pose2d bluePose, Pose2d redPose) {
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

    public double getPixelDropApproachHeading(AllianceColor allianceColor) {
        switch (this) {
            case BACKSTAGE_START:
                return Math.toRadians(180);
            case FRONTSTAGE_START:
            default:
                double targetHeading;
                switch (allianceColor) {
                    case RED:
                        targetHeading = Math.toRadians(270 - 135);
                        break;
                    default:
                    case BLUE:
                        targetHeading = Math.toRadians(90 + 135);
                        break;
                }
                return targetHeading;
        }
    }
}
