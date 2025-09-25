/* package org.firstinspires.ftc.teamcode.jules;

import com.bylazar.telemetry.PanelsTelemetry;          // <-- add
import com.bylazar.telemetry.TelemetryManager;        // <-- add
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.Metrics;

@TeleOp(name="Jules: RAM Tx Demo", group="Jules")
public class JulesTapDemo extends OpMode {
    DcMotorEx lf, rf;
    BNO055IMU imu;
    VoltageSensor vs;

    static final double TPR = 537.7;
    static final double W_IN = 3.78;
    static final double GR = 1.0;

    JulesTap tap;
    JulesRamTx tx;

    double lastEmit = 0;
    static final double PERIOD_S = 0.02;

    @Override public void init() {
        lf = hardwareMap.get(DcMotorEx.class,"leftFront");
        rf = hardwareMap.get(DcMotorEx.class,"rightFront");
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;   // Panels code expects degrees in many places
        imu.initialize(parameters);

        vs = hardwareMap.voltageSensor.iterator().next();

        tap = new JulesTap(TPR, W_IN, GR, vs, lf, rf);

        // ✅ Panels TelemetryManager + DS telemetry → Panels gets data
        TelemetryManager panelsTM = PanelsTelemetry.INSTANCE.getTelemetry();
        tx = new JulesRamTx(50_000, panelsTM, telemetry, "jules"); // ~16 min @ 50 Hz
    }

    @Override public void loop() {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        setDrivePower(power);
        tap.setLastCmd(power);

        double now = getRuntime();
        if (now - lastEmit >= PERIOD_S) {
            lastEmit = now;
            Metrics m = tap.sample(readHeadingDeg());
            tx.send(m); // RAM + Panels (via TelemetryManager)
        }
    }

    @Override public void stop() { try { tx.close(); } catch (Exception ignored) {} }

    private void setDrivePower(double p){ lf.setPower(p); rf.setPower(p); }
    private double readHeadingDeg(){ return imu.getAngularOrientation().firstAngle; }
}
/*