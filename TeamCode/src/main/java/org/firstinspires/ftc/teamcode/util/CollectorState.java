package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    COLLECTION(-32, 18, 0.86f),
    DRIVING_SAFE(-20, 4, 0.8f),
    SCORING(139, 0, 0.59f),
    SCORING_MEDIUM(131, 12, 0.529f),
    SCORING_HIGH(125, 18, 0.429f),
    FAR_COLLECTION(-15, 18, 0.78f),
    LOW_SCORING(140, 0, 0.35f),
    SAFE_POSITION(0, 0, 0.5f),
    HANG1(75,8, .7f),
    HANG2(75,0,.7f),
    HANG3(0,0,.7f);

    public final float WristPosition;
    public final int ArmPosition;

    public final int ExtenderPosition;

    public int ArmNudges = 0;
    public int WristNudges = 0;

    private CollectorState(int armPosition, int extenderPosition, float wristPosition) {
        this.ArmPosition = armPosition;
        this.ExtenderPosition = extenderPosition;
        this.WristPosition = wristPosition;
    }

}
