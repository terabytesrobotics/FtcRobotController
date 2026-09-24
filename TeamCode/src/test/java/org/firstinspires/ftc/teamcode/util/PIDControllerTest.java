package org.firstinspires.ftc.teamcode.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PIDControllerTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void integralUsesSeconds() {
        PIDController controller = new PIDController(0.0, 1.0, 0.0);

        assertEquals(1.0, controller.calculate(10.0, 0.0, 0.1), EPSILON);
        assertEquals(3.0, controller.calculate(10.0, 0.0, 0.2), EPSILON);
    }

    @Test
    public void derivativeUsesChangePerSecondAndResetClearsHistory() {
        PIDController controller = new PIDController(0.0, 0.0, 1.0);

        assertEquals(0.0, controller.calculate(10.0, 0.0, 0.1), EPSILON);
        assertEquals(100.0, controller.calculate(20.0, 0.0, 0.1), EPSILON);

        controller.reset();
        assertEquals(0.0, controller.calculate(20.0, 0.0, 0.1), EPSILON);
    }
}
