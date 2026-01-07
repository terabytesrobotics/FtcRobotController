package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
public class HeadlessConfig {
    // Driver forward-facing headings in degrees, field frame (CCW-positive).
    public static double blueForwardDeg = -90.0; // driver on +Y wall facing -Y
    public static double redForwardDeg = 90.0;   // driver on -Y wall facing +Y

    public static double forwardHeadingRadians(AllianceColor allianceColor) {
        return Math.toRadians(allianceColor == AllianceColor.BLUE ? blueForwardDeg : redForwardDeg);
    }
}
