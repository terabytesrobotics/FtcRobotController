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
        Platform.position = .3
        lLift.position = 0.918
        rLift.position = .98 - .918
        sleep(300)
    }
    fun pos2(){
        Platform.position = .27
        lLift.position = 0.91
        rLift.position = .98 - .91
        sleep(300)

    }
    fun pos3(){
        Platform.position = .257
        lLift.position = 0.9
        rLift.position = .98 - .9
        sleep(300)

    }
    fun pos4(){
        Platform.position = .36
        lLift.position = 0.871
        rLift.position = .98 - .871
        sleep(300)

    }
    fun pos5(){
        Platform.position = .38
        lLift.position = 0.885
        rLift.position = .98 - .885
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