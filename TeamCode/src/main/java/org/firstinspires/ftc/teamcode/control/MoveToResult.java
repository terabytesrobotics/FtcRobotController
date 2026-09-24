package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Observable state from one nonblocking move-to-pose update. */
public final class MoveToResult {
    private final Pose2D currentPose;
    private final Pose2D targetPose;
    private final double fieldXErrorMm;
    private final double fieldYErrorMm;
    private final double robotForwardErrorMm;
    private final double robotLeftErrorMm;
    private final double headingErrorRadians;
    private final DriveSignal requestedSignal;
    private final WheelPowers wheelPowers;
    private final boolean atTarget;

    public MoveToResult(
            Pose2D currentPose,
            Pose2D targetPose,
            double fieldXErrorMm,
            double fieldYErrorMm,
            double robotForwardErrorMm,
            double robotLeftErrorMm,
            double headingErrorRadians,
            DriveSignal requestedSignal,
            WheelPowers wheelPowers,
            boolean atTarget) {
        this.currentPose = currentPose;
        this.targetPose = targetPose;
        this.fieldXErrorMm = fieldXErrorMm;
        this.fieldYErrorMm = fieldYErrorMm;
        this.robotForwardErrorMm = robotForwardErrorMm;
        this.robotLeftErrorMm = robotLeftErrorMm;
        this.headingErrorRadians = headingErrorRadians;
        this.requestedSignal = requestedSignal;
        this.wheelPowers = wheelPowers;
        this.atTarget = atTarget;
    }

    public Pose2D getCurrentPose() { return currentPose; }
    public Pose2D getTargetPose() { return targetPose; }
    public double getFieldXErrorMm() { return fieldXErrorMm; }
    public double getFieldYErrorMm() { return fieldYErrorMm; }
    public double getRobotForwardErrorMm() { return robotForwardErrorMm; }
    public double getRobotLeftErrorMm() { return robotLeftErrorMm; }
    public double getHeadingErrorRadians() { return headingErrorRadians; }
    public double getDistanceErrorMm() { return Math.hypot(fieldXErrorMm, fieldYErrorMm); }
    public DriveSignal getRequestedSignal() { return requestedSignal; }
    public WheelPowers getWheelPowers() { return wheelPowers; }
    public boolean isAtTarget() { return atTarget; }
}
