package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry

class CLift(
        private val cLift: DcMotor
) {
    fun down(tics: Int, speed: Double){
        val currentPos = cLift.currentPosition
        val targetPos = currentPos - tics
        cLift.targetPosition = targetPos
        cLift.power = speed *-1
        while (cLift.isBusy) {

        }

    }
    fun up(tics: Int, speed: Double){
        val currentPos = cLift.currentPosition
        val targetPos = currentPos + tics
        cLift.targetPosition = targetPos
        cLift.power = speed
        while (cLift.isBusy) {

        }
        cLift.power = 0.0
    }
    init {


        cLift.targetPosition = 0
        cLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        cLift.mode = DcMotor.RunMode.RUN_TO_POSITION

    }
}