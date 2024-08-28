package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrueForTime {
    private final int millis;
    private final BooleanStateReader booleanStateReader;
    ElapsedTime timeSinceFalse = new ElapsedTime();

    public TrueForTime(int millis, BooleanStateReader buttonStateReader) {
        this.millis = millis;
        this.booleanStateReader = buttonStateReader;
    }

    public boolean evaluate() {
        boolean state = booleanStateReader.isActivated();
        if (!state) {
            timeSinceFalse.reset();
            return false;
        } else return timeSinceFalse.milliseconds() > millis;
    }
}
