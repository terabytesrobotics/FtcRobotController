package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DriveProfileTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void builderKeepsCriticalParametersNamedAndConvertsBoundaryUnits() {
        DriveProfile profile = DriveProfile.named("test")
                .maxTranslationPower(0.5)
                .maxRotationPower(0.25)
                .positionToleranceMm(12.0)
                .headingToleranceDeg(3.0)
                .settleTimeMs(200.0)
                .timeoutMs(4000.0)
                .build();

        assertEquals("test", profile.getName());
        assertEquals(0.5, profile.getMaxTranslationPower(), EPSILON);
        assertEquals(0.25, profile.getMaxRotationPower(), EPSILON);
        assertEquals(12.0, profile.getPositionToleranceMm(), EPSILON);
        assertEquals(Math.toRadians(3.0), profile.getHeadingToleranceRadians(), EPSILON);
        assertEquals(0.2, profile.getSettleTimeSeconds(), EPSILON);
        assertEquals(4.0, profile.getTimeoutSeconds(), EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsPowerOutsideMotorRange() {
        DriveProfile.named("invalid").maxTranslationPower(1.1);
    }
}
