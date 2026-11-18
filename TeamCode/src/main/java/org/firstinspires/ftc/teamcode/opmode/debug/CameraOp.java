package org.firstinspires.ftc.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Camera", group = "Debug")
public class CameraOp extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        robot.updateAprilTags();
    }
}
