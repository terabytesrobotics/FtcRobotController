package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

@TeleOp(name = "PinpointTest", group = "Starter Bot")
public class TestOp extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        robot.moveTo(300, 300);

        telemetry.addData("X coordinate (IN)", robot.getX());
        telemetry.addData("Y coordinate (IN)", robot.getY());
        telemetry.addData("Heading angle (DEGREES)", robot.getHeading());
    }
}
