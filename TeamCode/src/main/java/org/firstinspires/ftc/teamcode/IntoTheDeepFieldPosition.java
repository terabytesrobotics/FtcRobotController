package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public enum IntoTheDeepFieldPosition {
    // Start locations
    START_OBSERVATION_ZONE(-12, 62),
    START_MID(0, 62),
    START_NET_ZONE(36, 62),

    // Auton strategic points
    NET_PUSH_STAGING(36, 48),
    NET_PUSH_START_1(36, 0),
    NET_PUSH_START_2(54, 0),
    NET(56, 52),
    CLIP1_SCORE(0,32),

    // Part targets for who's expectd to be first or second to the spot
    PARK_TARGET_FIRST(-60, 50),
    PARK_TARGET_SECOND(-36, 54),
    PARK_TARGET_OUT_OF_WAY_OBS(-12, 35),
    PARK_TARGET_OUT_OF_WAY_NET(48, 12),

    OBS_WAYPOINT(-12, 38),
    MID_WAYPOINT(12, 38),
    NET_WAYPOINT(36, 38),

    // Teleop strategic points
    SUBMERSIBLE_APPROACH_ALLIANCE_SIDE(0, 48),
    SUBMERSIBLE_APPROACH_OPPONENT_SIDE(0, -48),
    SUBMERSIBLE_APPROACH_REAR_SIDE(48, 0),
    SUBMERSIBLE_APPROACH_AUDIENCE_SIDE(-48, 0),

    BASKET_APPROACH_FAST(48, 48),
    HIGH_BASKET_SCORING_APPROACH(56.5, 56.5),

    AUTON_PREP_BLOCK_NEUTRAL_1(48, 58),
    AUTON_PREP_BLOCK_NEUTRAL_2(58, 58),
    AUTON_PREP_BLOCK_NEUTRAL_3(45, 25.5),

    AUTON_BLOCK_NEUTRAL_1(48, 25.5),
    AUTON_BLOCK_NEUTRAL_2(58, 25.5),
    AUTON_BLOCK_NEUTRAL_3(68, 25.5);


    public final Vector2d BluePosition;
    public final Vector2d RedPosition;
    
    IntoTheDeepFieldPosition(double xBlue, double yBlue) {
        BluePosition = new Vector2d(xBlue, yBlue);
        RedPosition = new Vector2d(-xBlue, -yBlue);
    }

    public Vector2d getPosition(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? RedPosition : BluePosition;
    }
}
