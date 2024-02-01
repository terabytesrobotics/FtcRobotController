package org.firstinspires.ftc.teamcode.util;

public enum GreenGrabberState {
    GRABBED(0.1f),
    NOT_GRABBED(0.6f);

    public float ServoPosition;

    private GreenGrabberState(float servoPosition) {
        ServoPosition = servoPosition;
    }

    public GreenGrabberState toggle() {
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
