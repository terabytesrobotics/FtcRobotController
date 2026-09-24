package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

/** Monotonic loop timer with a cap that prevents one stalled loop from causing a large control jump. */
public final class LoopTimer {
    private final double maximumDtSeconds;
    private long lastTimeNanos;

    public LoopTimer(double maximumDtSeconds) {
        this.maximumDtSeconds = maximumDtSeconds;
    }

    public void reset() {
        lastTimeNanos = System.nanoTime();
    }

    public double nextSeconds() {
        long nowNanos = System.nanoTime();
        if (lastTimeNanos == 0L) {
            lastTimeNanos = nowNanos;
            return 0.0;
        }
        double dtSeconds = (nowNanos - lastTimeNanos) / 1_000_000_000.0;
        lastTimeNanos = nowNanos;
        return Range.clip(dtSeconds, 0.0, maximumDtSeconds);
    }
}
