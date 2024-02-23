package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.UpstageBackstageStart;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public enum NibusAutonomousPlan {
    START_BACKSTAGE(
            UpstageBackstageStart.BACKSTAGE_START,
            NibusAutonomousParkDirection.PARK_OUTSIDE),
    START_FRONTSTAGE(
            UpstageBackstageStart.FRONTSTAGE_START,
            NibusAutonomousParkDirection.PARK_INSIDE);

    public final UpstageBackstageStart StartingPosition;
    public final NibusAutonomousParkDirection ParkDirection;

    NibusAutonomousPlan(UpstageBackstageStart startingPosition, NibusAutonomousParkDirection parkDirection) {
        this.StartingPosition = startingPosition;
        this.ParkDirection = parkDirection;
    }

    public Pose2d pixelDropApproachPose(AllianceColor allianceColor, AlliancePropPosition detectedPosition) {
        double collectorHeading = getCollectorHeadingDuringPixelDrop(allianceColor, detectedPosition);
        Vector2d targetLocation = StartingPosition.getPixelTargetPosition(allianceColor, detectedPosition);
        Pose2d collectorPose = new Pose2d(targetLocation.getX(), targetLocation.getY(), collectorHeading);
        return NibusHelpers.robotPose2(collectorPose, NibusConstants.COLLECT_HEAD_BASE_OFFSET_X, 2, Math.PI);
    }

    public List<NibusCommand> afterScoringApproachCommands(AllianceColor allianceColor, CenterStageBackdropPosition backdropPosition) {
        double AUTON_SCORING_HEIGHT = 1;
        double APPROACH_DISTANCE = 3;
        double SAFTEY_DISTANCE = 6;

        ArrayList<NibusCommand> commands = new ArrayList<>();

        Mat.Tuple4<Double> scoringPositions = NibusHelpers.armExtenderWristAndOffsetForScoringHeight(AUTON_SCORING_HEIGHT);
        double defaultPreScoringOffset = scoringPositions.get_3();

        Pose2d preScoring = NibusHelpers.getPreScoringPose(allianceColor, backdropPosition, defaultPreScoringOffset + SAFTEY_DISTANCE, 2);
        Pose2d scoringPose = NibusHelpers.getPreScoringPose(allianceColor, backdropPosition, defaultPreScoringOffset - APPROACH_DISTANCE, 2);
        commands.add(NibusCommand.scoringHeightCommand(AUTON_SCORING_HEIGHT));
        commands.add(NibusCommand.driveDirectToPoseCommand(preScoring));
        commands.add(NibusCommand.driveDirectToPoseCommand(scoringPose));
        commands.add(NibusCommand.grabberStateCollectorStateCommand(BlueGrabberState.GRABBED, GreenGrabberState.NOT_GRABBED, CollectorState.DRIVING_SAFE));
        commands.add(NibusCommand.grabberStateCommand(BlueGrabberState.GRABBED, GreenGrabberState.GRABBED));
        return commands;
    }

    public List<NibusCommand> backStageScoringApproachCommands(AllianceColor allianceColor) {
        Vector2d scoringApproach = allianceColor.getScoringApproachLocation();

        Pose2d pose1 = new Pose2d(scoringApproach.getX(), scoringApproach.getY(), 0);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);

        ArrayList<NibusCommand> commands = new ArrayList<>();
        for (Pose2d pose: poses) {
            commands.add(NibusCommand.driveDirectToPoseFastCommand(pose));
        }
        return commands;
    }

    public List<NibusCommand> frontStageScoringApproachCommands(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        Vector2d audienceSideMiddleLane = allianceColor.getMiddleLaneAudienceWaypoint();
        Vector2d backstageSideMiddleLane = allianceColor.getMiddleLaneBackstageWaypoint();
        Vector2d scoringApproach = allianceColor.getScoringApproachLocation();

        double expectedRobotDropOrientation = Angle.norm(getCollectorHeadingDuringPixelDrop(allianceColor, alliancePropPosition) + Math.PI);
        double desiredTraverseLaneOrientation = Math.toRadians(0);
        double deltaFromDesired = Angle.normDelta(desiredTraverseLaneOrientation - expectedRobotDropOrientation);
        double audienceSideMiddleLaneOrientation = Angle.norm(expectedRobotDropOrientation + (deltaFromDesired / 2));
        double backstageSideMiddleLaneOrientation = Angle.norm(audienceSideMiddleLaneOrientation + (deltaFromDesired / 2));

        Pose2d pose1 = new Pose2d(audienceSideMiddleLane.getX(), audienceSideMiddleLane.getY(), audienceSideMiddleLaneOrientation);
        Pose2d pose2 = new Pose2d(scoringApproach.getX(), backstageSideMiddleLane.getY(), backstageSideMiddleLaneOrientation);
        Pose2d pose3 = new Pose2d(scoringApproach.getX(), scoringApproach.getY(), backstageSideMiddleLaneOrientation);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);
        poses.add(pose2);
        poses.add(pose3);

        ArrayList<NibusCommand> commands = new ArrayList<>();
        for (Pose2d pose: poses) {
            commands.add(NibusCommand.driveDirectToPoseFastCommand(pose));
        }
        return commands;
    }

    public List<NibusCommand> scoringCommands(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        List<NibusCommand> commands;
        switch (this) {
            case START_FRONTSTAGE:
                commands = frontStageScoringApproachCommands(allianceColor, alliancePropPosition);
                break;
            case START_BACKSTAGE:
            default:
                commands = backStageScoringApproachCommands(allianceColor);
        }
        commands.addAll(afterScoringApproachCommands(allianceColor, alliancePropPosition.backdropPosition()));
        return commands;
    }

    public Pose2d getParkPose(AllianceColor allianceColor) {
        Vector2d parkLocation = ParkDirection.getParkLocation(allianceColor);
        return new Pose2d(parkLocation.getX(), parkLocation.getY(), 0);
    }

    public List<NibusCommand> autonomousCommandsAfterPropDetect(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        Pose2d approachPose = pixelDropApproachPose(allianceColor, alliancePropPosition);
        double robotHeadingDuringPixelDrop = Angle.norm(
                getCollectorHeadingDuringPixelDrop(allianceColor, alliancePropPosition) + Math.PI);
        List<NibusCommand> prePlaceCommands = getPrePlaceCommands(allianceColor, robotHeadingDuringPixelDrop);
        List<NibusCommand> scoringCommands = scoringCommands(allianceColor, alliancePropPosition);

        List<NibusCommand> commands = new ArrayList<>(prePlaceCommands);
        commands.add(NibusCommand.driveDirectToPoseWithCollectorStatePreciseCommand(approachPose, CollectorState.COLLECTION));
        commands.add(NibusCommand.grabberStateCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED));
        commands.add(NibusCommand.collectorStateCommand(CollectorState.DRIVING_SAFE));
        commands.add(NibusCommand.grabberStateCommand(BlueGrabberState.GRABBED, GreenGrabberState.GRABBED));
        commands.addAll(scoringCommands);
        commands.add(NibusCommand.driveDirectToPoseCommand(getParkPose(allianceColor)));
        return commands;
    }

    private List<NibusCommand> getPrePlaceCommands(AllianceColor allianceColor, double robotHeadingDuringPixelDrop) {
        double INITIAL_NUDGE_Y = 2;
        double INITIAL_LATERAL_MOVE_X = 12;
        double INITIAL_FORWARD_MOVE_Y = 24;

        double xDirection = this == START_BACKSTAGE ? 1 : -1;
        double yDirection = allianceColor == AllianceColor.BLUE ? 1 : -1;
        Pose2d startingPosition = allianceColor == AllianceColor.BLUE ? StartingPosition.BluePose : StartingPosition.RedPose;

        List<NibusCommand> commands = new ArrayList<NibusCommand>();
        Pose2d pose0 = new Pose2d(
                startingPosition.getX(),
                startingPosition.getY() - (yDirection * INITIAL_NUDGE_Y),
                startingPosition.getHeading());

        Pose2d pose1 = new Pose2d(
                startingPosition.getX() + (xDirection * INITIAL_LATERAL_MOVE_X),
                startingPosition.getY(),
                startingPosition.getHeading());

        Pose2d pose2 = new Pose2d(
                pose1.getX(),
                pose1.getY() - (yDirection * INITIAL_FORWARD_MOVE_Y),
                robotHeadingDuringPixelDrop);

        commands.add(NibusCommand.driveDirectToPoseFastCommand(pose0));
        commands.add(NibusCommand.driveDirectToPoseFastCommand(pose1));
        commands.add(NibusCommand.driveDirectToPoseFastCommand(pose2));

        return commands;
    }

    public double getCollectorHeadingDuringPixelDrop(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        switch (this) {
            case START_FRONTSTAGE:
                switch (allianceColor) {
                    case RED:
                        switch (alliancePropPosition) {
                            case LEFT:
                                return (3 * Math.PI) / 2;
                            case MID:
                            default:
                                return ((3 * Math.PI) / 2) + (Math.PI / 4);
                            case RIGHT:
                                return 0;
                        }
                    case BLUE:
                    default:
                        switch (alliancePropPosition) {
                            case LEFT:
                                return 0;
                            case MID:
                            default:
                                return Math.PI / 4;
                            case RIGHT:
                                return Math.PI / 2;
                        }
                }
            case START_BACKSTAGE:
            default:
                return Math.PI;
/*                switch (allianceColor) {
                    case RED:
                        switch (alliancePropPosition) {
                            case LEFT:
                                return Math.PI;
                            case MID:
                            default:
                                return Math.PI + (Math.PI / 4);
                            case RIGHT:
                                return (3 * Math.PI) / 2;
                        }
                    case BLUE:
                    default:
                        switch (alliancePropPosition) {
                            case LEFT:
                                return Math.PI / 2;
                            case MID:
                            default:
                                return (Math.PI / 2) + (Math.PI / 4);
                            case RIGHT:
                                return Math.PI;
                        }
                }*/
        }
    }
}
