package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class MecanumMixerTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void mixesCardinalRobotDirectionsWithNamedSigns() {
        WheelPowers forward = MecanumMixer.mix(new DriveSignal(1.0, 0.0, 0.0));
        assertEquals(1.0, forward.getFrontLeft(), EPSILON);
        assertEquals(1.0, forward.getFrontRight(), EPSILON);
        assertEquals(1.0, forward.getBackLeft(), EPSILON);
        assertEquals(1.0, forward.getBackRight(), EPSILON);

        WheelPowers left = MecanumMixer.mix(new DriveSignal(0.0, 1.0, 0.0));
        assertEquals(-1.0, left.getFrontLeft(), EPSILON);
        assertEquals(1.0, left.getFrontRight(), EPSILON);
        assertEquals(1.0, left.getBackLeft(), EPSILON);
        assertEquals(-1.0, left.getBackRight(), EPSILON);

        WheelPowers counterclockwise = MecanumMixer.mix(new DriveSignal(0.0, 0.0, 1.0));
        assertEquals(-1.0, counterclockwise.getFrontLeft(), EPSILON);
        assertEquals(1.0, counterclockwise.getFrontRight(), EPSILON);
        assertEquals(-1.0, counterclockwise.getBackLeft(), EPSILON);
        assertEquals(1.0, counterclockwise.getBackRight(), EPSILON);
    }

    @Test
    public void normalizesCombinedRequestsWithoutChangingRatios() {
        WheelPowers powers = MecanumMixer.mix(new DriveSignal(1.0, 1.0, 1.0));
        assertTrue(Math.abs(powers.getFrontLeft()) <= 1.0);
        assertTrue(Math.abs(powers.getFrontRight()) <= 1.0);
        assertTrue(Math.abs(powers.getBackLeft()) <= 1.0);
        assertTrue(Math.abs(powers.getBackRight()) <= 1.0);
    }
}
