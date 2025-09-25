package org.firstinspires.ftc.teamcode.jules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class JulesTap {
    private final ElapsedTime clock = new ElapsedTime();
    private final DcMotorEx[] motors;
    private final VoltageSensor vs;
    private final double ticksPerRev, wheelDiameterIn, gearRatio;
    private volatile double lastCmd = 0.0;

    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    VoltageSensor battery, DcMotorEx... sampleMotors) {
        this.ticksPerRev     = ticksPerRev;
        this.wheelDiameterIn = wheelDiameterIn;
        this.gearRatio       = gearRatio;
        this.vs              = battery;
        this.motors          = sampleMotors;
        clock.reset();
    }

    /** Call this right where you command drive power so the log matches the command. */
    public void setLastCmd(double cmdPower) { this.lastCmd = clamp(cmdPower, -1.0, 1.0); }

    /** Sample once per loop (or at your panel tick). Provide heading in degrees. */
    public Metrics sample(double headingDeg) {
        Metrics m = new Metrics();
        m.t          = clock.seconds();
        m.cmdPower   = lastCmd;
        m.velIPS     = encodersToIPS();
        m.headingDeg = headingDeg;
        m.batteryV   = safeVoltage();
        return m;
    }

    // ----- Helpers -----
    private double encodersToIPS() {
        // REV getVelocity() returns ticks/second when RUN_USING_ENCODER
        double tps = 0.0;
        for (DcMotorEx m : motors) tps += m.getVelocity();
        tps /= Math.max(1, motors.length);
        double inchesPerTick = Math.PI * wheelDiameterIn * gearRatio / ticksPerRev;
        return tps * inchesPerTick;
    }
    private double safeVoltage() {
        try { return vs.getVoltage(); } catch (Exception e) { return 12.0; }
    }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
}
