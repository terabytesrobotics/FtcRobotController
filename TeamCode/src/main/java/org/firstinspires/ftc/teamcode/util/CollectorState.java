package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    COLLECTION(-32, 18, 0.86f),
    DRIVING_SAFE(-20, 4, 0.8f),
    SCORING(144, 0, 0.61f),
    SCORING_MEDIUM(131, 12, 0.53f),
    SCORING_HIGH(111, 18, 0.45f),
    FAR_COLLECTION(-15, 18, 0.78f),
    LOW_SCORING(140, 0, 0.35f),
    SAFE_POSITION(0, 0, 0.5f),
    HANG1(75,8, .7f),
    HANG2(75,0,.7f),
    HANG3(0,0,.7f);

    public float WristPosition;
    public int ArmPosition;

    public int ExtenderPosition;

    private CollectorState(int armPosition, int extenderPosition, float wristPosition) {
        this.ArmPosition = armPosition;
        this.ExtenderPosition = extenderPosition;
        this.WristPosition = wristPosition;
    }

}
