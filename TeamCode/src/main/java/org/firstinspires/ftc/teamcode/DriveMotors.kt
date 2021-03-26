package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor

class DriveMotors(
        private val flDcMotor: DcMotor,
        private val frDcMotor: DcMotor,
        private val blDcMotor: DcMotor,
        private val brDcMotor: DcMotor
) {

companion object{
    const val GEAR_RATIO = 15
    const val PPR = 145.1
    const val WHEEL_CIRCUMFRACE = 23.75
    const val PULSE_PER_ROTATION = PPR * GEAR_RATIO
    const val TICK_PER_INCH = PULSE_PER_ROTATION / WHEEL_CIRCUMFRACE
}

    fun direction(x: Double, y: Double, R: Double){
        val r = Math.hypot(-x, y)
        val robotAngle = Math.atan2(y, -x) - Math.PI / 4
        val rightX: Double = (R * -1).toDouble()
        val v1 = r * Math.cos(robotAngle) + rightX
        val v2 = r * Math.sin(robotAngle) - rightX
        val v3 = r * Math.sin(robotAngle) + rightX
        val v4 = r * Math.cos(robotAngle) - rightX

        flDcMotor.power = -v1
        frDcMotor.power = -v2
        blDcMotor.power = v3
        brDcMotor.power = -v4
    }
    fun forward(inches: Double) {
        val currentPos = frDcMotor.currentPosition
        val currentPos1 = flDcMotor.currentPosition
        val currentPos2 = brDcMotor.currentPosition
        val currentPos3 = blDcMotor.currentPosition
        val tics = totics(inches)
        val targetPos = currentPos + tics
        val targetPos1 = currentPos1 + tics
        val targetPos2 = currentPos2 + tics
        val targetPos3 = currentPos3 - tics
        frDcMotor.targetPosition = targetPos
        flDcMotor.targetPosition = targetPos1
        brDcMotor.targetPosition = targetPos2
        blDcMotor.targetPosition = targetPos3


        frDcMotor.power = 0.3
        flDcMotor.power = 0.3
        brDcMotor.power = 0.3
        blDcMotor.power = 0.3
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {

        }
        stop()
    }
    fun left(inches: Double) {
        val currentPos = frDcMotor.currentPosition
        val currentPos1 = flDcMotor.currentPosition
        val currentPos2 = brDcMotor.currentPosition
        val currentPos3 = blDcMotor.currentPosition
        val tics = totics(inches)
        val targetPos = currentPos + tics
        val targetPos1 = currentPos1 - tics
        val targetPos2 = currentPos2 - tics
        val targetPos3 = currentPos3 - tics
        frDcMotor.targetPosition = targetPos
        flDcMotor.targetPosition = targetPos1
        brDcMotor.targetPosition = targetPos2
        blDcMotor.targetPosition = targetPos3


        frDcMotor.power = 0.3
        flDcMotor.power = -0.3
        brDcMotor.power = -0.3
        blDcMotor.power = 0.3
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {
        }
        stop()
    }
    fun right(inches: Double) {
        val currentPos = frDcMotor.currentPosition
        val currentPos1 = flDcMotor.currentPosition
        val currentPos2 = brDcMotor.currentPosition
        val currentPos3 = blDcMotor.currentPosition
        val tics = totics(inches)
        val targetPos = currentPos - tics
        val targetPos1 = currentPos1 + tics
        val targetPos2 = currentPos2 + tics
        val targetPos3 = currentPos3 + tics
        frDcMotor.targetPosition = targetPos
        flDcMotor.targetPosition = targetPos1
        brDcMotor.targetPosition = targetPos2
        blDcMotor.targetPosition = targetPos3


        frDcMotor.power = -0.3
        flDcMotor.power = 0.3
        brDcMotor.power = 0.3
        blDcMotor.power = -0.3
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {
        }
        stop()
    }

    fun stop() {
        frDcMotor.power = 0.0
        flDcMotor.power = 0.0
        brDcMotor.power = 0.0
        blDcMotor.power = 0.0
    }

    private fun totics(inches: Double):Int{
        return (inches * TICK_PER_INCH).toInt()

    }

    init {

        frDcMotor.targetPosition = 0
        flDcMotor.targetPosition = 0
        brDcMotor.targetPosition = 0
        blDcMotor.targetPosition = 0

        flDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        flDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        frDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        blDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        blDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        brDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        brDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }
}