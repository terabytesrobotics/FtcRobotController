package org.firstinspires.ftc.teamcode;

class AppendageControlTarget {
    public double armTickTarget;
    public double extenderTickTarget;
    public double tiltTarget;
    public double wristTarget;
    public double pincerTarget;

    public AppendageControlTarget(double a, double e, double t, double w, double p) {
        this.armTickTarget = a;
        this.extenderTickTarget = e;
        this.tiltTarget = t;
        this.wristTarget = w;
        this.pincerTarget = p;
    }
}
