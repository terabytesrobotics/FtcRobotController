package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    CLOSE_COLLECTION(-20, 0, 0.75f),
    FAR_COLLECTION(-15, 18, 0.78f),
    DRIVING_SAFE(-10, 10, 0.80f),
    LOW_SCORING(140, 0, 0.35f),
    HIGH_SCORING(120, 18, 0.25f),
    SAFE_POSITION(0, 0, 0.5f),
    HANG1(75,8,.1f),
    HANG2(0,12,.1f),
    HANG3(0,0,.1f);

    public float WristPosition;
    public int ArmPosition;

    public int ExtenderPosition;

    private CollectorState(int armPosition, int extenderPosition, float wristPosition) {
        this.ArmPosition = armPosition;
        this.ExtenderPosition = extenderPosition;
        this.WristPosition = wristPosition;
    }

}
