package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {
    private static final String CENTER_COLLECTOR_NAME = "center_collector";
//    private static final String SIDE_INTAKE_NAME = "testIntake";
    private static final String SIDE_INTAKE_NAME = "servoTest";
//    private static final String RIGHT_INTAKE_NAME = "right_intake";

    private DcMotorEx centerCollector;
    private CRServo sideIntake;
//    private CRServo rightIntake;

    public Collector(HardwareMap hardwareMap) {
        centerCollector = hardwareMap.get(DcMotorEx.class, CENTER_COLLECTOR_NAME);
        sideIntake = hardwareMap.get(CRServo.class, SIDE_INTAKE_NAME);
//        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);
    }

    public void setCenterPower(double power) {
        centerCollector.setPower(power);
    }

    public void setSidePower(double power) {
        sideIntake.setPower(power);
//        rightIntake.setPower(power);
    }
}
