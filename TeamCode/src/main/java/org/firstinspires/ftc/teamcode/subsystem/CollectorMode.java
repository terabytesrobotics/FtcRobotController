package org.firstinspires.ftc.teamcode.subsystem;

public enum CollectorMode {
    OFF(0.0),
    INTAKE(1.0),
    EJECT(-1.0);

    private final double power;

    CollectorMode(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}
