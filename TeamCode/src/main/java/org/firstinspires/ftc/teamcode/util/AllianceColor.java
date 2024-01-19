package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CenterStageAprilTags;
import org.firstinspires.ftc.teamcode.CenterStageBackdropPosition;
import org.firstinspires.ftc.teamcode.NibusApproach;

public enum AllianceColor {
    BLUE(Math.toRadians(270)),
    RED(Math.toRadians(90));

    public double OperatorHeadingOffset;

    AllianceColor(double operatorHeadingOffset) {
        this.OperatorHeadingOffset = operatorHeadingOffset;
    }

    public Pose2d getAbsoluteFieldPose(UpstageBackstageStart alliancePose) {
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
                return new Vector2d(PointOfInterest.redGateAudience.X, PointOfInterest.redGateAudience.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueGateAudience.X, PointOfInterest.blueGateAudience.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getMiddleLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redGateBack.X, PointOfInterest.redGateBack.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueGateBack.X, PointOfInterest.blueGateBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getInnerLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redInnerBack.X, PointOfInterest.redInnerBack.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueInnerBack.X, PointOfInterest.blueInnerBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringPreApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redScoringPreApproach.X, PointOfInterest.redScoringPreApproach.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueScoringPreApprach.X, PointOfInterest.blueScoringPreApprach.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redScoringApproach.X, PointOfInterest.redScoringApproach.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueScoringApprach.X, PointOfInterest.blueScoringApprach.Y);
            default:
                return new Vector2d();
        }
    }

    public NibusApproach getScoringApproach() {
        switch (this) {
            case RED:
                return NibusApproach.RED_SCORING_APPROACH;
            case BLUE:
            default:
                return NibusApproach.BLUE_SCORING_APPROACH;
        }
    }

    public NibusApproach getMainCollectApproach() {
        switch (this) {
            case RED:
                return NibusApproach.RED_COLLECT_APPROACH;
            case BLUE:
            default:
                return NibusApproach.BLUE_COLLECT_APPROACH;
        }
    }

    public int getAprilTagId(AlliancePropPosition alliancePropPosition) {
        switch (this) {
            case RED:
                return alliancePropPosition.RedAprilTagId;
            case BLUE:
            default:
                return alliancePropPosition.BlueAprilTagId;
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
