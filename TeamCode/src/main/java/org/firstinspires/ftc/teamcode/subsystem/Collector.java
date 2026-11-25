package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {

    private CRServo centerCollector;

    public Collector(HardwareMap hardwareMap) {
        centerCollector = hardwareMap.get(CRServo.class, "centerCollector");
//        sideIntake = hardwareMap.get(CRServo.class, SIDE_INTAKE_NAME);
    }

    public void setCenterPower(double power) {
        centerCollector.setPower(power);
    }
}
