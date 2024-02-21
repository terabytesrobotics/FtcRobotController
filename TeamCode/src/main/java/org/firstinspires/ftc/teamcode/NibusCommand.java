package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

public class NibusCommand {
    public final Double ScoringHeight;
    public final CollectorState CollectorState;
    public final TrajectoryCreator TrajectoryCreator;
    public final BlueGrabberState BlueGrabberState;
    public final GreenGrabberState GreenGrabberState;
    public final Pose2d DriveDirectToPose;
    public final Double MinTimeMilis;

    public NibusCommand(
            Double scoringHeight,
            CollectorState collectorState,
            TrajectoryCreator trajectoryCreator,
            BlueGrabberState blueGrabberState,
            GreenGrabberState greenGrabberState,
            Pose2d driveDirectToPose,
            Double minTimeMilis) {
        ScoringHeight = scoringHeight;
        CollectorState = collectorState;
        TrajectoryCreator = trajectoryCreator;
        BlueGrabberState = blueGrabberState;
        GreenGrabberState = greenGrabberState;
        DriveDirectToPose = driveDirectToPose;
        MinTimeMilis = minTimeMilis;
    }

    public static NibusCommand trajectoryBuilderCommand(TrajectoryCreator trajectoryCreator) {
        return new NibusCommand(null, null, trajectoryCreator, null, null, null, 250d);
    }

    public static NibusCommand collectorStateCommand(CollectorState collectorState) {
        return new NibusCommand(null, collectorState, null, null, null, null, 250d);
    }

    public static NibusCommand driveDirectToPoseCommand(Pose2d pose) {
        return new NibusCommand(null, null, null, null, null, pose, 250d);
    }

    public static NibusCommand driveDirectToPoseWithCollectorStateCommand(Pose2d pose, CollectorState collectorState) {
        return new NibusCommand(null, collectorState, null, null, null, pose, 250d);
    }

    public static NibusCommand scoringHeightCommand(Double scoringHeight) {
        return new NibusCommand(scoringHeight, null, null, null, null, null, 250d);
    }

    public static NibusCommand grabberStateCommand(BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState) {
        return new NibusCommand(null, null, null, blueGrabberState, greenGrabberState, null, 250d);
    }

    public static NibusCommand waitCommand(Double waitMillis) {
        return new NibusCommand(null, null, null, null, null, null, waitMillis);
    }
}
