package org.firstinspires.ftc.teamcode.jules;

public final class Metrics {
    public double t;          // seconds since init
    public double cmdPower;   // -1..1
    public double velIPS;     // inches/second
    public double headingDeg; // degrees
    public double batteryV;   // volts
    public String label;      // optional tag

    // Odometry
    public double x;
    public double y;
    public double heading; // radians

    // IMU
    public double pitch;
    public double roll;
    public double yawRate;
    public double pitchRate;
    public double rollRate;
}