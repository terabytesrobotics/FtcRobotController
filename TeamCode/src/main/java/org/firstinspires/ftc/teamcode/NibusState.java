package org.firstinspires.ftc.teamcode;

public enum NibusState {
    MANUAL_DRIVE,
    DRIVE_DIRECT_TO_POSE,
    AUTONOMOUSLY_DRIVING,
    DETECT_POSE_FROM_APRIL_TAG,
    PROP_PIXEL_DROP,
    DETECT_ALLIANCE_MARKER,
    HALT_OPMODE;

    private NibusState() {
    }
}
