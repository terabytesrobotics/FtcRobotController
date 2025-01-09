package org.firstinspires.ftc.teamcode;

final class EndEffectorInitStage {
    final double tilt, wrist, pincer;
    final long durationMs;

    EndEffectorInitStage(double t, double w, double p, long d) {
        tilt = t;
        wrist = w;
        pincer = p;
        durationMs = d;
    }
}
