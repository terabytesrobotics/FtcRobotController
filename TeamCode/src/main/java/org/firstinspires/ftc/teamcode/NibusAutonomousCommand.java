package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

public class NibusAutonomousCommand {

    public final CollectorState CollectorState;
    public final TrajectoryCreator TrajectoryCreator;
    public final int MinTimeMillis;

    public final BlueGrabberState BlueGrabberState;
    public final GreenGrabberState GreenGrabberState;


    public NibusAutonomousCommand(TrajectoryCreator trajectoryCreator) {
        CollectorState = null;
        TrajectoryCreator = trajectoryCreator;
        MinTimeMillis = 250;
        BlueGrabberState = null;
        GreenGrabberState = null;
    }

    public NibusAutonomousCommand(CollectorState collectorState) {
        CollectorState = collectorState;
        TrajectoryCreator = null;
        MinTimeMillis = 250;
        BlueGrabberState = null;
        GreenGrabberState = null;
    }

    public NibusAutonomousCommand(BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState) {
        CollectorState = null;
        TrajectoryCreator = null;
        MinTimeMillis = 250;
        BlueGrabberState = blueGrabberState;
        GreenGrabberState = greenGrabberState;
    }
}
