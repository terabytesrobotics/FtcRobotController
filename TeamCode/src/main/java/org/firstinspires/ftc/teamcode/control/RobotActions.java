package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.CollectorMode;

/** Small behavior surface consumed by autonomous commands and usable by deterministic tests. */
public interface RobotActions {
    MoveToResult moveTo(Pose2D targetPose, DriveProfile profile, double dtSeconds);
    void resetMoveToControllers();
    void setCollectorMode(CollectorMode mode);
    void stopDrive();
    void stop();
}
