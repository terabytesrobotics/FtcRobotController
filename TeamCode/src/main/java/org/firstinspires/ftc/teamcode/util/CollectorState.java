package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    CLOSE_COLLECTION(-26, 0, 0.25f),
    FAR_COLLECTION(-20, 18, 0.8f),
    DRIVING_SAFE(0, 0, 0.5f),
    LOW_SCORING(170, 0, 0.8f),
    HIGH_SCORING(120, 18, 0.8f),
    SAFE_POSITION(0, 0, 0.5f);

    public float WristPosition;
    public int ArmPosition;

    public int ExtenderPosition;

    private CollectorState(int armPosition, int extenderPosition, float wristPosition) {
        this.ArmPosition = armPosition;
        this.ExtenderPosition = extenderPosition;
        this.WristPosition = wristPosition;
    }

}
