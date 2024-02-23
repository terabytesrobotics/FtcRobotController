package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CenterStageBackdropPosition;


public enum AlliancePropPosition {
    LEFT(1, 4,
            new Vector2d(22.5, 30),
            new Vector2d(2, -33),
            new Vector2d(-24, 34),
            new Vector2d(-48, -30)),
    MID(2, 5,
            new Vector2d(12, 24),
            new Vector2d(12, -24),
            new Vector2d(-36, 24),
            new Vector2d(-36, -24)),
    RIGHT(3, 6,
            new Vector2d(2, 34),
            new Vector2d(22.5, -30),
            new Vector2d(-48, 30),
            new Vector2d(-24, -34));

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

    public CenterStageBackdropPosition backdropPosition() {
        switch (this) {
            case LEFT:
                return CenterStageBackdropPosition.LEFT;
            case RIGHT:
                return CenterStageBackdropPosition.RIGHT;
            case MID:
            default:
                return  CenterStageBackdropPosition.CENTER;
        }
    }
}
