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

    public Pose2d pixelDropApproachPose(AllianceColor allianceColor, AlliancePropPosition detectedPosition, double collectorHeading) {
        //double LEFT_GRABBER_Y_OFFSET = 1;

        Vector2d targetLocation = StartingPosition.getPixelTargetPosition(allianceColor, detectedPosition);

        // TODO: This is a hack that's coupled to our specific approach direction
        // TODO: DOES NOT GENERALIZE
//        if (detectedPosition == AlliancePropPosition.MID) {
//            targetLocation = targetLocation.plus(new Vector2d(0, LEFT_GRABBER_Y_OFFSET));
//        }

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

        double expectedRobotDropOrientation = Angle.norm(getCollectorHeadingDuringPixelDrop(allianceColor) + Math.PI);
        double desiredTraverseLaneOrientation = Math.toRadians(0);
        double deltaFromDesired = Angle.normDelta(desiredTraverseLaneOrientation - expectedRobotDropOrientation);
        double audienceSideMiddleLaneOrientation = Angle.norm(expectedRobotDropOrientation + (deltaFromDesired / 2));
        double backstageSideMiddleLaneOrientation = Angle.norm(audienceSideMiddleLaneOrientation + (deltaFromDesired / 2));

        Pose2d pose1 = new Pose2d(audienceSideMiddleLane.getX(), audienceSideMiddleLane.getY(), audienceSideMiddleLaneOrientation);
        Pose2d pose2 = new Pose2d(scoringApproach.getX(), backstageSideMiddleLane.getY(), backstageSideMiddleLaneOrientation);
        Pose2d pose4 = new Pose2d(scoringApproach.getX(), scoringApproach.getY(), 0);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);
        poses.add(pose2);
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
        Pose2d approachPose = pixelDropApproachPose(allianceColor, alliancePropPosition, getCollectorHeadingDuringPixelDrop(allianceColor));
        List<NibusCommand> prePlaceCommands = getPrePlaceCommands(allianceColor);
        List<NibusCommand> afterPixelDropCommands = afterPixelDropCommands(allianceColor);

        List<NibusCommand> commands = new ArrayList<>(prePlaceCommands);
        commands.add(new NibusCommand(approachPose, CollectorState.COLLECTION));
        commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED));
        commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
        commands.addAll(afterPixelDropCommands);
        commands.add(new NibusCommand(CollectorState.SCORING));
        commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.NOT_GRABBED));
        commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
        commands.add(new NibusCommand(getParkPose(allianceColor)));
        return commands;
    }

    private List<NibusCommand> getPrePlaceCommands(AllianceColor allianceColor) {
        double INITIAL_NUDGE_Y = 2;
        double INITIAL_LATERAL_MOVE_X = 12;
        double INITIAL_FORWARD_MOVE_Y = 40;

        double xDirection = this == START_BACKSTAGE ? 1 : -1;
        double yDirection = allianceColor == AllianceColor.BLUE ? 1 : -1;
        double placementOrientation = Angle.norm(getCollectorHeadingDuringPixelDrop(allianceColor) + Math.PI);
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

    public double getCollectorHeadingDuringPixelDrop(AllianceColor allianceColor) {
        switch (this) {
            case START_FRONTSTAGE:
                switch (allianceColor) {
                    case RED:
                        return Math.toRadians(315);
                    case BLUE:
                    default:
                        return Math.toRadians(45);
                }
            case START_BACKSTAGE:
            default:
                switch (allianceColor) {
                    case RED:
                        return Math.toRadians(225);
                    case BLUE:
                    default:
                        return Math.toRadians(135);
                }
        }
    }
}
