package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

@FunctionalInterface
public interface TrajectoryCreator {
    public Trajectory create();
}
