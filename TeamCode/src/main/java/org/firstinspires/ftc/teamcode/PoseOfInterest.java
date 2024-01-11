package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum PoseOfInterest {
    RED_SCORING_APPROACH(new Pose2d(36, 36, 0)),
    RANDOM_POINT(new Pose2d(36, 0, 0)),
    BLUE_SCORING_APPROACH(new Pose2d(36, -36, 0));
    public final Pose2d Pose;

    PoseOfInterest(Pose2d pose) {
        this.Pose = pose;
    }

    public PoseOfInterest nextPose() {
        return PoseOfInterest.values()[(ordinal() + 1) % PoseOfInterest.values().length];
    }
}
