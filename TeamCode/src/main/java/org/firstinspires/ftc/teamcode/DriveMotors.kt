package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx


class DriveMotors(
        private val flDcMotor: DcMotorEx,
        private val frDcMotor: DcMotorEx,
        private val blDcMotor: DcMotorEx,
        private val brDcMotor: DcMotorEx
) {

companion object{
    const val GEAR_RATIO = 19.2
    const val PPR = 134.4
    const val WHEEL_CIRCUMFRACE = 11.875
    const val PULSE_PER_ROTATION = PPR * GEAR_RATIO
    const val TICK_PER_INCH = 42.6105
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
        runPos()
        flDcMotor.targetPositionTolerance = 5
        frDcMotor.targetPositionTolerance = 5
        blDcMotor.targetPositionTolerance = 5
        brDcMotor.targetPositionTolerance = 5

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
    fun forwardFAST(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 30
        frDcMotor.targetPositionTolerance = 30
        blDcMotor.targetPositionTolerance = 30
        brDcMotor.targetPositionTolerance = 30

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


        frDcMotor.power = 0.9
        flDcMotor.power = 0.9
        brDcMotor.power = 0.9
        blDcMotor.power = 0.9
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {

        }
        stop()
    }
    fun backward(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 5
        frDcMotor.targetPositionTolerance = 5
        blDcMotor.targetPositionTolerance = 5
        brDcMotor.targetPositionTolerance = 5

        val currentPos = frDcMotor.currentPosition
        val currentPos1 = flDcMotor.currentPosition
        val currentPos2 = brDcMotor.currentPosition
        val currentPos3 = blDcMotor.currentPosition
        val tics = totics(inches)
        val targetPos = currentPos - tics
        val targetPos1 = currentPos1 - tics
        val targetPos2 = currentPos2 - tics
        val targetPos3 = currentPos3 + tics
        frDcMotor.targetPosition = targetPos
        flDcMotor.targetPosition = targetPos1
        brDcMotor.targetPosition = targetPos2
        blDcMotor.targetPosition = targetPos3


        frDcMotor.power = -0.3
        flDcMotor.power = -0.3
        brDcMotor.power = -0.3
        blDcMotor.power = -0.3
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {

        }
        stop()
    }
    fun backFAST(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 30
        frDcMotor.targetPositionTolerance = 30
        blDcMotor.targetPositionTolerance = 30
        brDcMotor.targetPositionTolerance = 30

        val currentPos = frDcMotor.currentPosition
        val currentPos1 = flDcMotor.currentPosition
        val currentPos2 = brDcMotor.currentPosition
        val currentPos3 = blDcMotor.currentPosition
        val tics = totics(inches)
        val targetPos = currentPos - tics
        val targetPos1 = currentPos1 - tics
        val targetPos2 = currentPos2 - tics
        val targetPos3 = currentPos3 + tics
        frDcMotor.targetPosition = targetPos
        flDcMotor.targetPosition = targetPos1
        brDcMotor.targetPosition = targetPos2
        blDcMotor.targetPosition = targetPos3


        frDcMotor.power = -0.9
        flDcMotor.power = -0.9
        brDcMotor.power = -0.9
        blDcMotor.power = -0.9
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {

        }
        stop()
    }
    fun left(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 5
        frDcMotor.targetPositionTolerance = 5
        blDcMotor.targetPositionTolerance = 5
        brDcMotor.targetPositionTolerance = 5
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
    fun leftFast(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 30
        frDcMotor.targetPositionTolerance = 30
        blDcMotor.targetPositionTolerance = 30
        brDcMotor.targetPositionTolerance = 30
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


        frDcMotor.power = 0.9
        flDcMotor.power = -0.9
        brDcMotor.power = -0.9
        blDcMotor.power = 0.9
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {
        }
        stop()
    }
    fun right(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 5
        frDcMotor.targetPositionTolerance = 5
        blDcMotor.targetPositionTolerance = 5
        brDcMotor.targetPositionTolerance = 5

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
        blDcMotor.power = 0.3
        while (frDcMotor.isBusy||flDcMotor.isBusy||brDcMotor.isBusy||blDcMotor.isBusy) {
        }
        stop()
    }
    fun rightFAST(inches: Double) {
        runPos()
        flDcMotor.targetPositionTolerance = 30
        frDcMotor.targetPositionTolerance = 30
        blDcMotor.targetPositionTolerance = 30
        brDcMotor.targetPositionTolerance = 30

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


        frDcMotor.power = -0.9
        flDcMotor.power = 0.9
        brDcMotor.power = 0.9
        blDcMotor.power = 0.9
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





    fun backStupid(speed:Double){
            runEcode()
            frDcMotor.power = -speed
            flDcMotor.power = -speed
            brDcMotor.power = -speed
            blDcMotor.power = speed

        }
    fun forwardStupid(speed:Double){
        runEcode()
        frDcMotor.power = speed
        flDcMotor.power = speed
        brDcMotor.power = speed
        blDcMotor.power = -speed

    }

    fun runPos(){
        flDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        flDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        frDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        blDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        blDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        brDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        brDcMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }
    fun runEcode(){
        flDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        flDcMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frDcMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        blDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        blDcMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        brDcMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        brDcMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
    init {

        frDcMotor.targetPosition = 0
        flDcMotor.targetPosition = 0
        brDcMotor.targetPosition = 0
        blDcMotor.targetPosition = 0
        flDcMotor.targetPosition = 0
        brDcMotor.targetPosition = 0
        blDcMotor.targetPosition = 0
        runPos()
    }
    }