package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.CenterStageAprilTags;
import org.firstinspires.ftc.teamcode.CenterStageBackdropPosition;
import org.firstinspires.ftc.teamcode.NibusApproach;
import org.firstinspires.ftc.teamcode.TerabytesAutonomousPlan;

public enum AllianceColor {
    BLUE(Math.toRadians(0)),
    RED(Math.toRadians(0));

    public double OperatorHeadingOffset;

    AllianceColor(double operatorHeadingOffset) {
        this.OperatorHeadingOffset = operatorHeadingOffset;
    }

    public Pose2d getStartingPose(TerabytesAutonomousPlan autonomousPlan) {
        switch (this) {
            case RED:
                return new Pose2d(0, -62, (3.0 * Math.PI) / 2.0);
            case BLUE:
                return new Pose2d(0, 62, Math.PI / 2.0);
            default:
                return new Pose2d();
        }
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

    public RevBlinkinLedDriver.BlinkinPattern getAllianceColorBlinkinPattern() {
        switch (this) {
            case RED:
                return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            case BLUE:
            default:
                return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        }
    }

    public Pose2d getTeleopScoringApproachLocation() {
        switch (this) {
            case RED:
                return new Pose2d(PointOfInterest.RED_BACKDROP_APPROACH.X, PointOfInterest.RED_BACKDROP_APPROACH.Y, 0);
            case BLUE:
            default:
                return new Pose2d(PointOfInterest.BLUE_BACKDROP_APPROACH.X, PointOfInterest.BLUE_BACKDROP_APPROACH.Y, 0);
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
                    default:
                        return CenterStageAprilTags.BLUE_BACKDROP_RIGHT;
                }
            case RED:
            default:
                switch (position) {
                    case LEFT:
                        return CenterStageAprilTags.RED_BACKDROP_LEFT;
                    case CENTER:
                        return CenterStageAprilTags.RED_BACKDROP_CENTER;
                    case RIGHT:
                    default:
                        return CenterStageAprilTags.RED_BACKDROP_RIGHT;
                }
        }
    }

    public CenterStageAprilTags getAprilTagForBackdropApproach() {
        switch (this) {
            case BLUE:
                return CenterStageAprilTags.BLUE_BACKDROP_CENTER;
            case RED:
            default:
                return CenterStageAprilTags.RED_BACKDROP_CENTER;
        }
    }
}
