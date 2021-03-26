package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

class ShooterCommands(
        private val Shooter: DcMotor,
        private val Platform: Servo,
        private val lLift: Servo,
        private val rLift: Servo,
        private val LFinger: Servo,
        private val RFinger: Servo
) {
    fun pos1(){
        Platform.position = .36
        lLift.position = 0.899
        rLift.position = .98 - .899
        sleep(300)
    }
    fun pos2(){
        Platform.position = .325
        lLift.position = 0.899
        rLift.position = .98 - .899
        sleep(300)

    }
    fun pos3(){
        Platform.position = .29
        lLift.position = 0.91
        rLift.position = .98 - .9
        sleep(300)

    }
    fun pos4(){
        Platform.position = .475
        lLift.position = .855
        rLift.position = .124
        sleep(300)

    }
    fun shoot(){
            LFinger.position = 0.31
            RFinger.position = 0.0
            sleep(800)
            RFinger.position = 0.26
            LFinger.position = 0.05
            sleep(300)

    }
    init {
        Shooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}