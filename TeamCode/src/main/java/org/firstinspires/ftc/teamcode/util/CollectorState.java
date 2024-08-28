package org.firstinspires.ftc.teamcode.util;

public enum CollectorState {
    COLLECTION(-32, 7.09f, 0.86f, 0),
    DRIVING_SAFE(-20, 1.57f, 0.8f, 0),
    SCORING(139, 0, 0.59f, 0),
    SCORING_MEDIUM(131, 4.72f, 0.529f, 0),
    SCORING_HIGH(125, 7.09f, 0.429f, 0),
    FAR_COLLECTION(-15, 7.09f, 0.78f, 0),
    LOW_SCORING(140, 0, 0.35f, 0),
    SAFE_POSITION(0, 0, 0.5f, 0),
    HANG1(75,3.15f, .7f, 0),
    HANG2(75,0,.7f, 0),
    HANG3(0,0,.7f, 0),
    SCORING_0(158, 0, .63f, 24),
    SCORING_1(150, 1.18f, .61f, 24),
    SCORING_2(138, 3.15f, .57f, 24),
    SCORING_3(126, 5.51f, .53f, 24),
    SCORING_4(113, 7.09f, .39f, 20);

    public final float WristPosition;
    public final int ArmPosition;

    public final float ExtenderPosition;
    public final int AppendageOffset;

    public int ArmNudges = 0;
    public int WristNudges = 0;

    private CollectorState(int armPosition, float extenderPosition, float wristPosition, int appendageOffset) {
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
