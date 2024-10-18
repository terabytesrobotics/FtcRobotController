package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum IntoTheDeepFieldPosition {
    AUTON_START_MID(0, 62),
    AUTON_PRE_NEUTRAL_PUSH_STAGING(36, 48),
    AUTON_PRE_NEUTRAL_PUSH_1(36, 0),
    AUTON_PRE_NEUTRAL_PUSH_2(54, 0),
    AUTON_NEUTRAL_PUSH_TARGET(56, 52),
    AUTON_PARK_TARGET(-56, 50);

    public final Vector2d BluePosition;
    public final Vector2d RedPosition;
    
    IntoTheDeepFieldPosition(double xBlue, double yBlue) {
        BluePosition = new Vector2d(xBlue, yBlue);
        RedPosition = new Vector2d(-xBlue, -yBlue);
    }
}
