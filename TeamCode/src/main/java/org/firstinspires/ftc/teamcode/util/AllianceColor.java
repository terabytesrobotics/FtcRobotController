package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CenterStageAprilTags;
import org.firstinspires.ftc.teamcode.CenterStageBackdropPosition;

public enum AllianceColor {
    BLUE(Math.toRadians(270)),
    RED(Math.toRadians(90));

    public double OperatorHeadingOffset;

    AllianceColor(double operatorHeadingOffset) {
        this.OperatorHeadingOffset = operatorHeadingOffset;
    }

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

    public Vector2d getMiddleLaneAudienceWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(pointsOfInterest.redGateAudience.X, pointsOfInterest.redGateAudience.Y);
            case BLUE:
                return new Vector2d(pointsOfInterest.blueGateAudience.X, pointsOfInterest.blueGateAudience.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getMiddleLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(pointsOfInterest.redGateBack.X, pointsOfInterest.redGateBack.Y);
            case BLUE:
                return new Vector2d(pointsOfInterest.blueGateBack.X, pointsOfInterest.blueGateBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getInnerLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(pointsOfInterest.redInnerBack.X, pointsOfInterest.redInnerBack.Y);
            case BLUE:
                return new Vector2d(pointsOfInterest.blueInnerBack.X, pointsOfInterest.blueInnerBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringPreApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(pointsOfInterest.redScoringPreApproach.X, pointsOfInterest.redScoringPreApproach.Y);
            case BLUE:
                return new Vector2d(pointsOfInterest.blueScoringPreApprach.X, pointsOfInterest.blueScoringPreApprach.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(pointsOfInterest.redScoringApproach.X, pointsOfInterest.redScoringApproach.Y);
            case BLUE:
                return new Vector2d(pointsOfInterest.blueScoringApprach.X, pointsOfInterest.blueScoringApprach.Y);
            default:
                return new Vector2d();
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

    public CenterStageAprilTags getAprilTagForScoringPosition(CenterStageBackdropPosition position) {
        switch (this) {
            case BLUE:
                switch (position) {
                    case LEFT:
                        return CenterStageAprilTags.BLUE_BACKDROP_LEFT;
                    case CENTER:
                        return CenterStageAprilTags.BLUE_BACKDROP_CENTER;
                    case RIGHT:
                        return CenterStageAprilTags.BLUE_BACKDROP_RIGHT;
                }
                break;
            case RED:
                switch (position) {
                    case LEFT:
                        return CenterStageAprilTags.RED_BACKDROP_LEFT;
                    case CENTER:
                        return CenterStageAprilTags.RED_BACKDROP_CENTER;
                    case RIGHT:
                        return CenterStageAprilTags.RED_BACKDROP_RIGHT;
                }
            default:
                return null;
        }
        return null;
    }
}
