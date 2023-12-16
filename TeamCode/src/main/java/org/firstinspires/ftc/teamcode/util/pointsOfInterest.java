package org.firstinspires.ftc.teamcode.util;

public enum pointsOfInterest {
    blueOuterAudience(-36, 60),
    blueOuterBack(12, 60),
    blueInnerAudience(-36,36),
    blueInnerBack(12, 36),
    blueGateAudience(-36, 12),
    blueGateBack(12,12),
    redGateAudience(-36, -12),
    redGateBack(12,-12),
    redInnerAudience(-36, -36),
    redInnerBack(12,-36),
    redOuterAudience(-36, -60),
    redOuterBack(12, -60),
    redDiagonalAudience(-48, 24),
    redDiagonalBack(24,-24),
    blueDiagonalAudience(-48, -24),
    blueDiagonalBack(24,24),
    midGateAuidience(-36,0),
    midGateBack(24,0),
    redScoringApproach(48, -36),
    blueScoringApprach(48, 36),
    redLeftBackstagePark(50, -12),
    redRightBackstagePark(50, -60),
    blueRightBackstagePark(50, 12),
    blueLeftBackstagePark(50, 60);






    public double X;
    public double Y;

    pointsOfInterest(double x, double y) {
        X = x;
        Y = y;
    }

}
