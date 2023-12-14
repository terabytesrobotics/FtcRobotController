package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AllianceLocation;
import org.firstinspires.ftc.teamcode.util.AlliancePose;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;

public abstract class TeleOpNibus2000 extends LinearOpMode {

    private final AllianceColor allianceColor;
    private final AlliancePose startPose;

    public TeleOpNibus2000(AllianceColor allianceColor, AlliancePose startPose) {
        super();
        this.allianceColor = allianceColor;
        this.startPose = startPose;
    }

    @Override
    public void runOpMode() {
        Nibus2000 nibus = new Nibus2000(
                gamepad1,
                gamepad2,
                hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        nibus.init(allianceColor.getAbsoluteFieldPose(startPose));
        waitForStart();
        nibus.startup();
        while (!isStopRequested()) {
            nibus.evaluate();
        }
    }
}
