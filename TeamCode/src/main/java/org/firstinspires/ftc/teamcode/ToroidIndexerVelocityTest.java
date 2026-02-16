package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Toroid Indexer Velocity Test")
public class ToroidIndexerVelocityTest extends LinearOpMode {
    public static String MOTOR_NAME = "coreHex";
    public static double TRANSIT_RPM = 60.0;
    public static double SHOOT_RPM = 180.0;
    public static double TRANSIT_RPM_STEP = 2.5;
    public static double SHOOT_RPM_STEP = 5.0;
    public static double MAX_RPM = 200.0;
    public static double MIN_RPM = 0.0;
    public static boolean CCW_IS_POSITIVE = true;
    public static boolean TRANSIT_CCW = false;
    public static boolean SHOOT_CCW = true;
    public static double INTAKE_POWER = 0.6;
    public static double COAST_BEFORE_BRAKE_SEC = 0.15;
    public static double BRAKE_BEFORE_REVERSE_SEC = 0.1;
    public static double VELOCITY_SIGN_THRESHOLD_TPS = 40.0;

    private static final double TICKS_PER_REV = 288.0;

    private enum Mode {
        TRANSIT,
        SHOOT,
        STOP
    }

    private enum TransitionPhase {
        NONE,
        COAST,
        BRAKE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spinMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Mode mode = Mode.STOP;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastRb = false;
        boolean lastLb = false;
        boolean intakeEnabled = false;
        TransitionPhase transitionPhase = TransitionPhase.NONE;
        ElapsedTime transitionTimer = new ElapsedTime();
        int lastTargetSign = 0;
        double pendingTargetRpm = 0.0;
        int pendingTargetSign = 0;
        telemetry.addLine("hold y=transit (slow), hold b=shoot (fast), hold a=stop");
        telemetry.addLine("dpad left/right adjusts transit RPM");
        telemetry.addLine("left stick button runs intake forward; dpad down runs intake forward");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.y) {
                mode = Mode.TRANSIT;
            } else if (gamepad2.b) {
                mode = Mode.SHOOT;
            } else if (gamepad2.a) {
                mode = Mode.STOP;
            } else {
                mode = Mode.STOP;
            }

            intakeEnabled = gamepad2.left_stick_button;

            if (gamepad2.dpad_right && !lastDpadRight) {
                TRANSIT_RPM = Range.clip(TRANSIT_RPM + TRANSIT_RPM_STEP, MIN_RPM, MAX_RPM);
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                TRANSIT_RPM = Range.clip(TRANSIT_RPM - TRANSIT_RPM_STEP, MIN_RPM, MAX_RPM);
            }
            if (gamepad2.right_bumper && !lastRb) {
                SHOOT_RPM = Range.clip(SHOOT_RPM + SHOOT_RPM_STEP, MIN_RPM, MAX_RPM);
            }
            if (gamepad2.left_bumper && !lastLb) {
                SHOOT_RPM = Range.clip(SHOOT_RPM - SHOOT_RPM_STEP, MIN_RPM, MAX_RPM);
            }

            lastDpadLeft = gamepad2.dpad_left;
            lastDpadRight = gamepad2.dpad_right;
            lastRb = gamepad2.right_bumper;
            lastLb = gamepad2.left_bumper;

            double ccwSign = CCW_IS_POSITIVE ? 1.0 : -1.0;
            double targetRpm = 0.0;
            double targetTicksPerSec = 0.0;
            if (spinMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            switch (mode) {
                case TRANSIT:
                    targetRpm = (TRANSIT_CCW ? ccwSign : -ccwSign) * TRANSIT_RPM;
                    break;
                case SHOOT:
                    targetRpm = (SHOOT_CCW ? ccwSign : -ccwSign) * SHOOT_RPM;
                    break;
                case STOP:
                default:
                    targetRpm = 0.0;
                    break;
            }

            int targetSign = targetRpm == 0.0 ? 0 : (targetRpm > 0.0 ? 1 : -1);
            double actualTps = spinMotor.getVelocity();
            int actualSign = Math.abs(actualTps) > VELOCITY_SIGN_THRESHOLD_TPS ? (actualTps > 0.0 ? 1 : -1) : 0;
            boolean reversing = targetSign != 0 && actualSign != 0 && targetSign != actualSign;
            pendingTargetRpm = targetRpm;
            pendingTargetSign = targetSign;
            if (transitionPhase == TransitionPhase.NONE && reversing) {
                transitionPhase = TransitionPhase.COAST;
                transitionTimer.reset();
            }

            targetTicksPerSec = rpmToTicksPerSec(pendingTargetRpm);
            if (transitionPhase == TransitionPhase.COAST) {
                spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                spinMotor.setPower(0.0);
                if (transitionTimer.seconds() >= COAST_BEFORE_BRAKE_SEC) {
                    transitionPhase = TransitionPhase.BRAKE;
                    transitionTimer.reset();
                }
            } else if (transitionPhase == TransitionPhase.BRAKE) {
                spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                spinMotor.setPower(0.0);
                if (transitionTimer.seconds() >= BRAKE_BEFORE_REVERSE_SEC) {
                    transitionPhase = TransitionPhase.NONE;
                }
            } else {
                spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                spinMotor.setVelocity(targetTicksPerSec);
                lastTargetSign = pendingTargetSign;
            }

            double intakePower = 0.0;
            if (gamepad2.dpad_down) {
                intakePower = INTAKE_POWER;
            } else if (intakeEnabled) {
                intakePower = INTAKE_POWER;
            }
            intakeMotor.setPower(intakePower);

            telemetry.addData("Mode", mode);
            telemetry.addData("CCW_IS_POSITIVE", CCW_IS_POSITIVE);
            telemetry.addData("Transit RPM", TRANSIT_RPM);
            telemetry.addData("Shoot RPM", SHOOT_RPM);
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Target tps", targetTicksPerSec);
            telemetry.addData("Actual tps", spinMotor.getVelocity());
            telemetry.addData("EncoderTicks", spinMotor.getCurrentPosition());
            telemetry.addData("MotorMode", spinMotor.getMode());
            telemetry.addData("Transition", transitionPhase);
            telemetry.addData("IntakeEnabled", intakeEnabled);
            telemetry.addData("IntakePower", intakePower);
            telemetry.update();
        }
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

}
