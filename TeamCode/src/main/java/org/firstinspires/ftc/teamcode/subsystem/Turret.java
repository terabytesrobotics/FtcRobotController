package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MathHelper;

public class Turret {
    private final double maxTurretRot = 350;
    private static final String rotationName = "shooterRot1";

    private Servo rotationServo;

    public Turret(HardwareMap hardwareMap) {
        rotationServo = hardwareMap.get(Servo.class, rotationName);
    }

    public boolean rotateTo(double deg, Telemetry t) {
        double pwm = (deg / maxTurretRot + 1) / 2;
        pwm = Math.max(.05, Math.min(pwm, .95));
        rotationServo.setPosition(pwm);
        t.addData("pwm", pwm);

        // TODO: return whether at target
        return false;
    }
}
