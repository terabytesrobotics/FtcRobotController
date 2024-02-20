package org.firstinspires.ftc.teamcode.util;

public enum BlueGrabberState {
    GRABBED(0.05f),
    NOT_GRABBED(0.65f);

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
