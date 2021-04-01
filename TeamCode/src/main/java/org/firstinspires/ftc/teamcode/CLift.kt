package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry

class CLift(
        private val cLift: DcMotor,
        private val TopLimit: RevTouchSensor,
        private val BottomLimit: RevTouchSensor,

        ) {
    fun down(tics: Int, speed: Double){
        cLift.mode = DcMotor.RunMode.RUN_TO_POSITION
        val currentPos = cLift.currentPosition
        val targetPos = currentPos + tics
        cLift.targetPosition = targetPos
        cLift.power = speed
        while (cLift.isBusy) {

        }

    }
    fun up(tics: Int, speed: Double){
        cLift.mode = DcMotor.RunMode.RUN_TO_POSITION
        val currentPos = cLift.currentPosition
        val targetPos = currentPos - tics
        cLift.targetPosition = targetPos
        cLift.power = speed*-1
        while (cLift.isBusy) {

        }
        cLift.power = 0.0
    }
    fun fullDown():Boolean {
        cLift.mode = DcMotor.RunMode.RUN_USING_ENCODER
        if (TopLimit.value == 0.0) {
            cLift.power = 1.0
            return false

        } else {
            cLift.power = 0.0
            return true

        }

    }
    fun fullUp():Boolean{
        cLift.mode = DcMotor.RunMode.RUN_USING_ENCODER
        cLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        if (BottomLimit.value == 0.0) {
            cLift.power = -1.0
            return false

        } else {
            cLift.power = 0.0
            return true

        }
    }

    init {


        cLift.targetPosition = 0
        cLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        cLift.mode = DcMotor.RunMode.RUN_TO_POSITION

    }
}