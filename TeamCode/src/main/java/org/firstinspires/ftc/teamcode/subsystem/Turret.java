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

    // servo to turret is 1:1.16
    public boolean rotateTo(double deg, Telemetry t) {
        deg = Math.max(-203, Math.min(deg, 203));
        double servoDeg = deg / 1.16;
        double pwm = (servoDeg + maxTurretRot / 2) / maxTurretRot;
        pwm = Math.max(0.05, Math.min(pwm, 0.95));
        rotationServo.setPosition(pwm);

        t.addData("target turret angle", deg);
        t.addData("servo angle", servoDeg);
        t.addData("pwm", pwm);

        return false;
    }
}
