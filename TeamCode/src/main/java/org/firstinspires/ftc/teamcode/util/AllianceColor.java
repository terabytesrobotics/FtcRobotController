package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
}
