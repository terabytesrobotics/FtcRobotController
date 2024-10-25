package org.firstinspires.ftc.teamcode;

public enum IntoTheDeepOpModeState {
    // Autonomous only
    COMMAND_SEQUENCE,

    // Tele-op only
    HEADLESS_DRIVE,
    RADIAL_DRIVING_MODE,
    SUBMERSIBLE_APPROACH,
    BASKET_APPROACH,
    STOPPED_UNTIL_END,
    HALT_OPMODE
}
