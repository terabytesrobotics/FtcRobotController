package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class IntoTheDeepCollectPosition {

    public Pose2d CollectPose;
    public double HeightSignal;
    public double DistanceSignal;
    public double WristSignal;

    public IntoTheDeepCollectPosition(
            Pose2d collectPose,
            double heightSignal,
            double distanceSignal,
            double wristSignal
    ) {
        this.CollectPose = collectPose;
        this.HeightSignal = heightSignal;
        this.DistanceSignal = distanceSignal;
        this.WristSignal = wristSignal;
    }
}
