package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

class ShooterCommands(
        private val Shooter: DcMotorEx,
        private val Platform: Servo,
        private val lLift: Servo,
        private val rLift: Servo,
        private val LFinger: Servo,
        private val RFinger: Servo
) {
    fun pos1(){
        Platform.position = .3
        lLift.position = 0.898
        rLift.position = .98 - .918
        sleep(300)
    }
    fun pos2(){
        Platform.position = .27
        lLift.position = 0.89
        rLift.position = .98 - .91
        sleep(300)

    }
    fun pos3(){
        Platform.position = .257
        lLift.position = 0.88
        rLift.position = .98 - .9
        sleep(300)

    }
    fun pos4(){
        Platform.position = .362
        lLift.position = 0.845
        rLift.position = .98 - .855
        sleep(300)

    }
    fun pos5(){
        Platform.position = .38
        lLift.position = 0.865
        rLift.position = .98 - .885
        sleep(300)

    }
    fun shoot(){
            LFinger.position = 0.31
            RFinger.position = 0.0
            sleep(500)
            RFinger.position = 0.24
            LFinger.position = 0.07
            sleep(700)

    }
    init {
        Shooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}