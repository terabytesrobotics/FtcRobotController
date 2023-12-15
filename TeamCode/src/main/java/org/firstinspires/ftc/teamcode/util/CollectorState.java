package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    CLOSE_COLLECTION(-20, 0, 0.25f),
    FAR_COLLECTION(-15, 18, 0.22f),
    DRIVING_SAFE(-20, 0, 0.5f),
    LOW_SCORING(140, 0, 0.65f),
    HIGH_SCORING(120, 18, 0.75f),
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
