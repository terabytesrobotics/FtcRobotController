package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.TeleOpNibus2000;

public enum BlueGrabberState {
    GRABBED(0.65f),
    NOT_GRABBED(0.9f);

    public float ServoPosition;

    private BlueGrabberState(float servoPosition) {
        ServoPosition = servoPosition;
    }

    public BlueGrabberState toggle() {
        switch (this) {
            case NOT_GRABBED:
                return GRABBED;
            case GRABBED:
                return NOT_GRABBED;
            default:
                return GRABBED;
        }
    }
}
