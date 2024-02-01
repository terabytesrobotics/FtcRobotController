package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    COLLECTION(-32, 18, 0.86f, 0),
    DRIVING_SAFE(-20, 4, 0.8f, 0),
    SCORING(139, 0, 0.59f, 0),
    SCORING_MEDIUM(131, 12, 0.529f, 0),
    SCORING_HIGH(125, 18, 0.429f, 0),
    FAR_COLLECTION(-15, 18, 0.78f, 0),
    LOW_SCORING(140, 0, 0.35f, 0),
    SAFE_POSITION(0, 0, 0.5f, 0),
    HANG1(75,8, .7f, 0),
    HANG2(75,0,.7f, 0),
    HANG3(0,0,.7f, 0),
    SCORING_0(158, 0, .63f, 24),
    SCORING_1(150, 3, .61f, 24),
    SCORING_2(138, 8, .57f, 24),
    SCORING_3(126, 14, .53f, 24),
    SCORING_4(113, 18, .39f, 20);

    public final float WristPosition;
    public final int ArmPosition;

    public final int ExtenderPosition;
    public final int AppendageOffset;

    public int ArmNudges = 0;
    public int WristNudges = 0;

    private CollectorState(int armPosition, int extenderPosition, float wristPosition, int appendageOffset) {
        this.ArmPosition = armPosition;
        this.ExtenderPosition = extenderPosition;
        this.WristPosition = wristPosition;
        this.AppendageOffset = appendageOffset;
    }

    public static CollectorState[] scoringStates() {
        return new CollectorState[] {
                SCORING_0,
                SCORING_1,
                SCORING_2,
                SCORING_3,
                SCORING_4
        };
    }
}
