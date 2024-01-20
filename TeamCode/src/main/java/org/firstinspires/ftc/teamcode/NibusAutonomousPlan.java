package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.UpstageBackstageStart;

import java.util.ArrayList;
import java.util.List;

public enum NibusAutonomousPlan {
    START_BACKSTAGE(
            Math.toRadians(180),
            UpstageBackstageStart.BACKSTAGE_START,
            NibusAutonomousParkDirection.PARK_INSIDE),
    START_FRONTSTAGE(
            Math.toRadians(0),
            UpstageBackstageStart.FRONTSTAGE_START,
            NibusAutonomousParkDirection.PARK_OUTSIDE);

    public final double CollectorHeadingDuringPixelDrop;
    public final UpstageBackstageStart StartingPosition;
    public final NibusAutonomousParkDirection ParkDirection;

    NibusAutonomousPlan(double collectorHeadingDuringPixelDrop, UpstageBackstageStart startingPosition, NibusAutonomousParkDirection parkDirection) {
        this.CollectorHeadingDuringPixelDrop = collectorHeadingDuringPixelDrop;
        this.StartingPosition = startingPosition;
        this.ParkDirection = parkDirection;
    }

    public Pose2d pixelDropApproachPose(AllianceColor allianceColor, AlliancePropPosition detectedPosition, double collectorHeading) {
        Vector2d targetLocation = StartingPosition.getPixelTargetPosition(allianceColor, detectedPosition);
        double placementOrientation = Angle.norm(collectorHeading + Math.PI);
        Pose2d targetPose = new Pose2d(targetLocation.getX(), targetLocation.getY(), placementOrientation);
        return NibusHelpers.collectApproachPose(targetPose);
    }

    public List<NibusCommand> backStageAfterPixelDropCommands(AllianceColor allianceColor) {
        Vector2d scoringApproach = allianceColor.getScoringApproachLocation();

        Pose2d pose1 = new Pose2d(scoringApproach.getX(), scoringApproach.getY(), 0);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);

        ArrayList<NibusCommand> commands = new ArrayList<>();
        for (Pose2d pose: poses) {
            commands.add(new NibusCommand(pose));
        }
        return commands;
    }

    public List<NibusCommand> frontStageAfterPixelDropCommands(AllianceColor allianceColor) {
        Vector2d audienceSideMiddleLane = allianceColor.getMiddleLaneAudienceWaypoint();
        Vector2d backstageSideMiddleLane = allianceColor.getMiddleLaneBackstageWaypoint();
        Vector2d scoringApproach = allianceColor.getScoringApproachLocation();

        double expectedDropOrientation = Math.toRadians(180);
        double rotationDirection = allianceColor == AllianceColor.BLUE ? 1 : -1;
        double audienceSideMiddleLaneOrientation = Angle.norm(expectedDropOrientation + (rotationDirection * Math.PI / 2));
        double backstageSideMiddleLaneOrientation = Angle.norm(audienceSideMiddleLaneOrientation + (rotationDirection * Math.PI / 2));

        Pose2d pose1 = new Pose2d(audienceSideMiddleLane.getX(), audienceSideMiddleLane.getY(), audienceSideMiddleLaneOrientation);
        Pose2d pose2 = new Pose2d(backstageSideMiddleLane.getX(), backstageSideMiddleLane.getY(), backstageSideMiddleLaneOrientation);
        Pose2d pose3 = new Pose2d(backstageSideMiddleLane.getX(), scoringApproach.getY(), backstageSideMiddleLaneOrientation);
        Pose2d pose4 = new Pose2d(scoringApproach.getX(), scoringApproach.getY(), 0);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);
        poses.add(pose2);
        poses.add(pose3);
        poses.add(pose4);

        ArrayList<NibusCommand> commands = new ArrayList<>();
        for (Pose2d pose: poses) {
            commands.add(new NibusCommand(pose));
        }
        return commands;
    }

    public List<NibusCommand> afterPixelDropCommands(AllianceColor allianceColor) {
        switch (this) {
            case START_FRONTSTAGE:
                return frontStageAfterPixelDropCommands(allianceColor);
            case START_BACKSTAGE:
            default:
                return backStageAfterPixelDropCommands(allianceColor);
        }
    }

    public Pose2d getParkPose(AllianceColor allianceColor) {
        Vector2d parkLocation = ParkDirection.getParkLocation(allianceColor);
        return new Pose2d(parkLocation.getX(), parkLocation.getY(), 0);
    }

    public List<NibusCommand> autonomousCommandsAfterPropDetect(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        Pose2d approachPose = pixelDropApproachPose(allianceColor, alliancePropPosition, CollectorHeadingDuringPixelDrop);
        List<NibusCommand> prePlaceCommands = getPrePlaceCommands(allianceColor);
        List<NibusCommand> afterPixelDropCommands = afterPixelDropCommands(allianceColor);

        List<NibusCommand> commands = new ArrayList<>(prePlaceCommands);
        commands.add(new NibusCommand(approachPose, CollectorState.COLLECTION));
        commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED));
        commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
//        commands.addAll(afterPixelDropCommands);
//        commands.add(new NibusCommand(CollectorState.SCORING));
//        commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.NOT_GRABBED));
//        commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
//        commands.add(new NibusCommand(getParkPose(allianceColor)));
        return commands;
    }

    private List<NibusCommand> getPrePlaceCommands(AllianceColor allianceColor) {
        double INITIAL_NUDGE_Y = 2;
        double INITIAL_LATERAL_MOVE_X = 12;
        double INITIAL_FORWARD_MOVE_Y = 20;

        double xDirection = this == START_BACKSTAGE ? 1 : -1;
        double yDirection = allianceColor == AllianceColor.BLUE ? 1 : -1;
        double placementOrientation = Angle.norm(CollectorHeadingDuringPixelDrop + Math.PI);
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
                startingPosition.getHeading());

        Pose2d pose3 = new Pose2d(
                pose2.getX(),
                pose2.getY(),
                placementOrientation);

        commands.add(new NibusCommand(pose0));
        commands.add(new NibusCommand(pose1));
        commands.add(new NibusCommand(pose2));
        commands.add(new NibusCommand(pose3));

        return commands;
    }
}
