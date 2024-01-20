package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.UpstageBackstageStart;

import java.util.ArrayList;
import java.util.List;

public enum NibusAutonomousPlan {
    START_BACKSTAGE(UpstageBackstageStart.BACKSTAGE_START, NibusAutonomousParkDirection.PARK_INSIDE),
    START_UPSTAGE(UpstageBackstageStart.FRONTSTAGE_START, NibusAutonomousParkDirection.PARK_OUTSIDE);

    public final UpstageBackstageStart StartingPosition;
    public final NibusAutonomousParkDirection ParkDirection;

    NibusAutonomousPlan(UpstageBackstageStart startingPosition, NibusAutonomousParkDirection parkDirection) {
        this.StartingPosition = startingPosition;
        this.ParkDirection = parkDirection;
    }

    public Pose2d pixelDropApproachPose(AllianceColor allianceColor, AlliancePropPosition detectedPosition) {
        Vector2d targetLocation = StartingPosition.getPixelTargetPosition(allianceColor, detectedPosition);
        double placementOrientation = this == NibusAutonomousPlan.START_BACKSTAGE ? Math.toRadians(180) : Math.toRadians(0);
        Pose2d targetPose = new Pose2d(targetLocation.getX(), targetLocation.getY(), placementOrientation);
        return NibusHelpers.collectApproachPose(targetPose);
    }

    public List<NibusCommand> afterPixelDropCommands(AllianceColor allianceColor) {
        Vector2d waypoint1 = allianceColor.getMiddleLaneAudienceWaypoint();
        Vector2d waypoint2 = allianceColor.getScoringPreApproachLocation();
        Vector2d waypoint3 = allianceColor.getScoringApproachLocation();

        Pose2d pose1 = new Pose2d(waypoint1.getX(), waypoint1.getY(), 0);
        Pose2d pose2 = new Pose2d(waypoint2.getX(), waypoint2.getY(), 0);
        Pose2d pose3 = new Pose2d(waypoint3.getX(), waypoint3.getY(), 0);

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose1);
        poses.add(pose2);
        poses.add(pose3);

        ArrayList<NibusCommand> commands = new ArrayList<>();
        for (Pose2d pose: poses) {
            commands.add(new NibusCommand(pose));
        }
        return commands;
    }

    public Pose2d getParkPose(AllianceColor allianceColor) {
        Vector2d parkLocation = ParkDirection.getParkLocation(allianceColor);
        return new Pose2d(parkLocation.getX(), parkLocation.getY(), 0);
    }

    public List<NibusCommand> autonomousCommandsAfterPropDetect(AllianceColor allianceColor, AlliancePropPosition alliancePropPosition) {
        Pose2d approachPose = pixelDropApproachPose(allianceColor, alliancePropPosition);

        List<NibusCommand> prePlaceCommands = getPrePlaceCommands(allianceColor);
        List<NibusCommand> afterPixelDropCommands = afterPixelDropCommands(allianceColor);

        List<NibusCommand> commands = new ArrayList<NibusCommand>();
        commands.addAll(prePlaceCommands);
//        commands.add(new NibusCommand(CollectorState.COLLECTION));
//        commands.add(new NibusCommand(approachPose));
//        commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED));
//        commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
        //commands.addAll(afterPixelDropCommands);
        //commands.add(new NibusCommand(CollectorState.SCORING));
        //commands.add(new NibusCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.NOT_GRABBED));
        //commands.add(new NibusCommand(CollectorState.DRIVING_SAFE));
        //commands.add(new NibusCommand(getParkPose(allianceColor)));
        return commands;
    }

    private List<NibusCommand> getPrePlaceCommands(AllianceColor allianceColor) {

        double INITIAL_LATERAL_MOVE_X = 20;
        double INITIAL_FORWARD_MOVE_Y = 20;

        double yDirection = allianceColor == AllianceColor.BLUE ? 1 : -1;
        double xDirection = this == NibusAutonomousPlan.START_BACKSTAGE ? 1 : -1;
        double facePlacementOrientation = this == NibusAutonomousPlan.START_BACKSTAGE ? Math.toRadians(0) : Math.toRadians(180);
        Pose2d startingPosition = allianceColor == AllianceColor.BLUE ? StartingPosition.BluePose : StartingPosition.RedPose;

        List<NibusCommand> commands = new ArrayList<NibusCommand>();
        Pose2d pose1 = new Pose2d(
                startingPosition.getX() + (xDirection * INITIAL_LATERAL_MOVE_X),
                startingPosition.getY(),
                startingPosition.getHeading());

        Pose2d pose2 = new Pose2d(
                pose1.getX(),
                pose1.getY(),
                facePlacementOrientation);

        Pose2d pose3 = new Pose2d(
                pose2.getX(),
                pose2.getY() + (yDirection * INITIAL_FORWARD_MOVE_Y),
                facePlacementOrientation);

        Log.d("autonomousPlan", String.format("%.1f", pose1.getX()));
        Log.d("autonomousPlan", String.format("%.1f", pose1.getY()));
        Log.d("autonomousPlan", String.format("%.1f", Math.toDegrees(pose1.getHeading())));
        commands.add(new NibusCommand(pose1));
        //commands.add(new NibusCommand(pose2));
        //commands.add(new NibusCommand(pose3));

        return commands;
    }
}
