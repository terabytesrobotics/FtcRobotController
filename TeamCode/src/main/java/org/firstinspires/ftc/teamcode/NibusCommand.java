package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

public class NibusCommand {

    public final CollectorState CollectorState;
    public final TrajectoryCreator TrajectoryCreator;
    public final int MinTimeMillis;

    public final BlueGrabberState BlueGrabberState;
    public final GreenGrabberState GreenGrabberState;
    public final Pose2d DriveDirectToPose;

    public NibusCommand(TrajectoryCreator trajectoryCreator) {
        CollectorState = null;
        TrajectoryCreator = trajectoryCreator;
        MinTimeMillis = 250;
        BlueGrabberState = null;
        GreenGrabberState = null;
        DriveDirectToPose = null;
    }

    public NibusCommand(CollectorState collectorState) {
        CollectorState = collectorState;
        TrajectoryCreator = null;
        MinTimeMillis = 250;
        BlueGrabberState = null;
        GreenGrabberState = null;
        DriveDirectToPose = null;
    }

    public NibusCommand(BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState) {
        CollectorState = null;
        TrajectoryCreator = null;
        MinTimeMillis = 250;
        BlueGrabberState = blueGrabberState;
        GreenGrabberState = greenGrabberState;
        DriveDirectToPose = null;
    }

    public NibusCommand(Pose2d driveDirectToPose) {
        CollectorState = null;
        TrajectoryCreator = null;
        MinTimeMillis = 250;
        BlueGrabberState = null;
        GreenGrabberState = null;
        DriveDirectToPose = driveDirectToPose;
    }
}
