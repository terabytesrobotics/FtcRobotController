package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.data.Node;

import java.util.List;

public class DashboardTelemetryWrapper extends MultipleTelemetry {
    private TelemetryPacket packet = new TelemetryPacket();
    private static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final String FIELD_IMAGE = "/biobuzz-field.webp";

    public DashboardTelemetryWrapper(Telemetry telemetry, Telemetry dashboardTelemetry) {
        super(telemetry, dashboardTelemetry);
    }

    @Override
    public Item addData(String s, Object o) {
        packet.put(s, o);
        return super.addData(s, o);
    }

    public void draw(double currentX, double currentY, double currentHeading) {
        Canvas field = packet.fieldOverlay();

        drawPose(field, currentX / Constants.IN_TO_MM, currentY / Constants.IN_TO_MM,
                currentHeading, ROBOT_RADIUS_INCHES, "#2196F3");
    }

    private void drawPose(Canvas field, double x, double y,
                          double heading, double radius, String color) {
        // field is 144 inches long/wide
        field.drawImage(FIELD_IMAGE, 0.0, 0.0, 144.0, 144.0);
        field.setStroke(color);
        field.strokeCircle(x, y, radius);

        field.strokeLine(x, y, x + radius * Math.cos(heading), y + radius * Math.sin(heading));
    }

    public void drawPath(List<Node> path) {
        if (path == null || path.size() < 2) return;

        Canvas field = packet.fieldOverlay();

        field.setStroke("#FF9800");

        for (int i = 0; i < path.size() - 1; i++) {
            Node a = path.get(i);
            Node b = path.get(i + 1);

            field.strokeLine(
                    a.x / Constants.IN_TO_MM,
                    a.y / Constants.IN_TO_MM,
                    b.x / Constants.IN_TO_MM,
                    b.y / Constants.IN_TO_MM
            );
        }
    }

    public void updateDashboard() {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
