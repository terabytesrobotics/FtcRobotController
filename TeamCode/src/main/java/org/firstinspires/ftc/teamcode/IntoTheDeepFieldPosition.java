package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum IntoTheDeepFieldPosition {
    // Start locations
    START_OBSERVATION_ZONE(-24, 62),
    START_MID(0, 62),
    START_NET_ZONE(36, 62),

    // Auton strategic points
    NET_PUSH_STAGING(36, 48),
    NET_PUSH_START_1(36, 0),
    NET_PUSH_START_2(54, 0),
    NET(56, 52),

    // Part targets for who's expectd to be first or second to the spot
    PARK_TARGET_FIRST(-60, 50),
    PARK_TARGET_SECOND(-48, 60),

    // Teleop strategic points
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
