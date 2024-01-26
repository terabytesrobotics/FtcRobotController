package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;


public enum AlliancePropPosition {
    LEFT(1, 4,
            new Vector2d(25, 30),
            new Vector2d(0, -30),
            new Vector2d(-25, 30),
            new Vector2d(-48, -30)),
    MID(2, 5,
            new Vector2d(12, 24),
            new Vector2d(12, -24),
            new Vector2d(-36, 24),
            new Vector2d(-36, -24)),
    RIGHT(3, 6,
            new Vector2d(0, 30),
            new Vector2d(25, -30),
            new Vector2d(-48, 30),
            new Vector2d(-25, -30));

    public int BlueAprilTagId;
    public int RedAprilTagId;
    public Vector2d BlueBackstagePixelTarget;
    public Vector2d RedBackstagePixelTarget;
    public Vector2d BlueFrontstagePixelTarget;
    public Vector2d RedFrontstagePixelTarget;

    AlliancePropPosition(
            int blueAprilTagId,
            int redAprilTagId,
            Vector2d blueBackstagePixelTarget,
            Vector2d redBackstagePixelTarget,
            Vector2d blueFrontstagePixelTarget,
            Vector2d redFrontstagePixelTarget) {
        BlueAprilTagId = blueAprilTagId;
        RedAprilTagId = redAprilTagId;
        BlueBackstagePixelTarget = blueBackstagePixelTarget;
        RedBackstagePixelTarget = redBackstagePixelTarget;
        BlueFrontstagePixelTarget = blueFrontstagePixelTarget;
        RedFrontstagePixelTarget = redFrontstagePixelTarget;
    }
}
