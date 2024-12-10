package org.firstinspires.ftc.teamcode;

public enum IntoTheDeepOpModeState {
    // Autonomous only
    INITIALIZE_TELEOP,
    MANUAL_CONTROL,
    COMMAND_SEQUENCE,
    // Tele-op only
    RADIAL_DRIVING_MODE,
    SUBMERSIBLE_APPROACH,
    BASKET_APPROACH,
    STOPPED_UNTIL_END,
    HALT_OPMODE
}
