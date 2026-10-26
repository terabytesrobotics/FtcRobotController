package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.TestSubsystem;

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
