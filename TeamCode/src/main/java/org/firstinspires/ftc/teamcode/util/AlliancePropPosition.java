package org.firstinspires.ftc.teamcode.util;

import org.intellij.lang.annotations.JdkConstants;

public enum AlliancePropPosition {
    LEFT(1, 4),
    MID(2, 5),
    RIGHT(3, 6);

    public int BlueAprilTagId;
    public int RedAprilTagId;

    AlliancePropPosition(int blueAprilTagId, int redAprilTagId) {
        BlueAprilTagId = blueAprilTagId;
        RedAprilTagId = redAprilTagId;
    }
}
