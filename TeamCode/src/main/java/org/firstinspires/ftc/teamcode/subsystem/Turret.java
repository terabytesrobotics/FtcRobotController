package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private final double maxTurretRot = 350;
    private static final String rotationName = "shooterRot1";

    private Servo rotationServo;

    public Turret(HardwareMap hardwareMap) {
        rotationServo = hardwareMap.get(Servo.class, rotationName);
    }

    public boolean rotateTo(double deg) {
        double pwm = deg / maxTurretRot;
        rotationServo.setPosition(Math.max(-.95, Math.min(pwm, .95)));

        // TODO: return whether at target
        return false;
    }
}
