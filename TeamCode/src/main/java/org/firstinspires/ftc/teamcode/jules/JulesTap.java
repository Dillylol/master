package org.firstinspires.ftc.teamcode.jules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.pedropathing.follower.Follower; // For odometry pose

import java.util.List;

public class JulesTap {
    private final ElapsedTime clock = new ElapsedTime();
    private final DcMotorEx[] motors;
    private final VoltageSensor vs;
    private final double ticksPerRev;

    // These are no longer final so they can be updated live
    private double wheelDiameterIn;
    private double gearRatio;

    private volatile double lastCmd = 0.0;

    // --- Main Constructor ---
    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    VoltageSensor battery, DcMotorEx... sampleMotors) {
        this.ticksPerRev = ticksPerRev;
        this.vs = battery;
        this.motors = sampleMotors;
        // Set the initial values using our new updatable method
        updateConstants(wheelDiameterIn, gearRatio);
        clock.reset();
    }

    // --- Constructor overload to accept a List of motors ---
    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    VoltageSensor battery, List<DcMotorEx> sampleMotors) {
        this(ticksPerRev, wheelDiameterIn, gearRatio, battery, sampleMotors.toArray(new DcMotorEx[0]));
    }

    /**
     * Updates the physical constants used for velocity calculations.
     * Call this in your loop to ensure the tap is using the latest tuned values.
     */
    public void updateConstants(double wheelDiameterIn, double gearRatio) {
        this.wheelDiameterIn = wheelDiameterIn;
        this.gearRatio = gearRatio;
    }

    /** Call this right where you command drive power so the log matches the command. */
    public void setLastCmd(double cmdPower) { this.lastCmd = clamp(cmdPower, -1.0, 1.0); }

    /**
     * Samples all robot metrics.
     * This is the primary data collection method.
     * @param imu The robot's IMU for orientation and angular velocity.
     * @param follower The Pedro Pathing Follower for odometry pose.
     * @return A Metrics object populated with the latest data.
     */
    public Metrics sample(IMU imu, Follower follower) {
        Metrics m = new Metrics();
        m.t = clock.seconds();
        m.cmdPower = lastCmd;
        m.velIPS = encodersToIPS();
        m.batteryV = safeVoltage();

        // --- NEW: Add Full Odometry and IMU Data ---
        if (follower != null) {
            m.x = follower.getPose().getX();
            m.y = follower.getPose().getY();
            m.heading = follower.getPose().getHeading();
        }

        if (imu != null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            m.headingDeg = orientation.getYaw(AngleUnit.DEGREES);
            m.pitch = orientation.getPitch(AngleUnit.DEGREES);
            m.roll = orientation.getRoll(AngleUnit.DEGREES);
            m.yawRate = angularVelocity.zRotationRate;
            m.pitchRate = angularVelocity.xRotationRate;
            m.rollRate = angularVelocity.yRotationRate;
        }
        return m;
    }

    /** Overloaded sample method for tests that don't need IMU/Odo. */
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
        // This calculation now uses the updatable member variables
        double tps = 0.0;
        for (DcMotorEx m : motors) {
            tps += m.getVelocity();
        }
        tps /= Math.max(1, motors.length);
        double inchesPerTick = (Math.PI * this.wheelDiameterIn) / (this.ticksPerRev * this.gearRatio);
        return tps * inchesPerTick;
    }

    private double safeVoltage() {
        try { return vs.getVoltage(); } catch (Exception e) { return 12.0; }
    }

    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
}