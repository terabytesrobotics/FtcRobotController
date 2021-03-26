package org.firstinspires.ftc.teamcode

import android.os.SystemClock.sleep
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry

class Timer {

fun wait(count: Int, mills:Long) {

    for (i in 0..count) {
        sleep(mills)
        telemetry.addData("wait", i)
    }
}
}