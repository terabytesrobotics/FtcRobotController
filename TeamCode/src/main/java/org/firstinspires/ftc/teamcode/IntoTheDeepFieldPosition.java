package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum IntoTheDeepFieldPosition {
    AUTON_START_MID(0, 62),
    AUTON_PRE_NEUTRAL_PUSH_STAGING(36, 48),
    AUTON_PRE_NEUTRAL_PUSH_1(36, 0),
    AUTON_PRE_NEUTRAL_PUSH_2(54, 0),
    AUTON_NEUTRAL_PUSH_TARGET(56, 52),
    AUTON_PARK_TARGET(-56, 50),

    SUBMERSIBLE_APPROACH_ALLIANCE_SIDE(0, 48),
    SUBMERSIBLE_APPROACH_OPPONENT_SIDE(0, -48),
    SUBMERSIBLE_APPROACH_REAR_SIDE(48, 0),
    SUBMERSIBLE_APPROACH_AUDIENCE_SIDE(-48, 0),

    BASKET_APPROACH(48, 48);

    public final Vector2d BluePosition;
    public final Vector2d RedPosition;
    
    IntoTheDeepFieldPosition(double xBlue, double yBlue) {
        BluePosition = new Vector2d(xBlue, yBlue);
        RedPosition = new Vector2d(-xBlue, -yBlue);
    }
}
