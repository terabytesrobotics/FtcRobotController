package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DashboardTelemetryWrapper extends MultipleTelemetry {
    TelemetryPacket packet = null;
    private static final double MM_PER_INCH = 25.4;
    private static final double ROBOT_RADIUS_INCHES = 9.0;

    public DashboardTelemetryWrapper(Telemetry telemetry, Telemetry dashboardTelemetry) {
        super(telemetry, dashboardTelemetry);
    }

    @Override
    public Item addData(String s, Object o) {
        if (packet == null) {
            packet = new TelemetryPacket();
        }

        packet.put(s, o);

        return super.addData(s, o);
    }

    public void updateDashboard() {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        packet = null;
    }

    public void draw(double currentX, double currentY, double currentHeading) {
        Canvas field = packet.fieldOverlay();
        drawPose(field, currentX / MM_PER_INCH, currentY / MM_PER_INCH,
                currentHeading, ROBOT_RADIUS_INCHES, "#2196F3");
//        drawPose(field, targetX / MM_PER_INCH, targetY / MM_PER_INCH,
//                targetRot, 3.0, "#FF9800");
    }

    private void drawPose(Canvas field, double x, double y, double heading, double radius, String color) {
        field.setStroke(color);
        field.strokeCircle(x, y, radius);
        field.strokeLine(x, y, x + radius * Math.cos(heading), y + radius * Math.sin(heading));
    }
}
