package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.command.DriveToCommand;
import org.firstinspires.ftc.teamcode.command.SequentialCommandRunner;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.DriveProfile;

/** Field plans are kept outside Robot so strategy can change without changing hardware behavior. */
public final class StarterAutoPlans {
    private static final double TEST_DRIVE_DISTANCE_MM = 300.0;

    private static final DriveProfile TEST_TRAVEL = DriveProfile.named("test-travel")
            .maxTranslationPower(0.45)
            .maxRotationPower(0.35)
            .positionToleranceMm(20.0)
            .headingToleranceDeg(4.0)
            .settleTimeMs(200.0)
            .timeoutMs(5000.0)
            .build();

    private static final DriveProfile TEST_RETURN_PRECISE = DriveProfile.named("test-return-precise")
            .maxTranslationPower(0.35)
            .maxRotationPower(0.30)
            .positionToleranceMm(10.0)
            .headingToleranceDeg(2.0)
            .settleTimeMs(300.0)
            .timeoutMs(5000.0)
            .build();

    private StarterAutoPlans() {
    }

    public static SequentialCommandRunner driveOutAndBack(Pose2D startPose) {
        Pose2D driveOutPose = offsetForward(startPose, TEST_DRIVE_DISTANCE_MM);

        return SequentialCommandRunner.sequence(
                WaitCommand.milliseconds(250.0),
                DriveToCommand.to(driveOutPose).using(TEST_TRAVEL),
                WaitCommand.milliseconds(250.0),
                DriveToCommand.to(startPose).using(TEST_RETURN_PRECISE));
    }

    private static Pose2D offsetForward(Pose2D pose, double forwardMm) {
        double headingRadians = pose.getHeading(AngleUnit.RADIANS);
        double x = pose.getX(DistanceUnit.MM) + Math.cos(headingRadians) * forwardMm;
        double y = pose.getY(DistanceUnit.MM) + Math.sin(headingRadians) * forwardMm;
        return new Pose2D(
                DistanceUnit.MM, x, y,
                AngleUnit.RADIANS, headingRadians);
    }
}
