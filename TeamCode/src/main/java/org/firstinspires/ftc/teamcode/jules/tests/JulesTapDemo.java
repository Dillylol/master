package org.firstinspires.ftc.teamcode.jules.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import android.content.Context;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.Metrics;

// ---- JULES bridge bits ----
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;

import java.io.IOException;

@TeleOp(name="Jules: RAM Tx Demo (HTTP Stream)", group="Jules")
public class JulesTapDemo extends OpMode {
    DcMotorEx lf, rf;
    IMU imu;
    VoltageSensor vs;

    static final double TPR = 537.7;
    static final double W_IN = 3.78;
    static final double GR = 1.0;

    JulesTap tap;
    JulesRamTx tx;

    // --- streaming + dump pieces ---
    private JulesBuffer buffer;                  // batch/dump storage
    private JulesStreamBus streamBus;            // live stream bus (SSE)
    private JulesHttpBridge http;                // NanoHTTPD bridge

    double lastEmit = 0;
    static final double PERIOD_S = 0.02;         // ~50 Hz

    @Override public void init() {
        lf = hardwareMap.get(DcMotorEx.class,"leftFront");
        rf = hardwareMap.get(DcMotorEx.class,"rightFront");
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // *** IMPORTANT: You must update the directions below to match your robot's hub mounting!
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        vs = hardwareMap.voltageSensor.iterator().next();

        tap = new JulesTap(TPR, W_IN, GR, vs, lf, rf);

        // ✅ Panels TelemetryManager + DS telemetry → Panels gets data
        TelemetryManager panelsTM = PanelsTelemetry.INSTANCE.getTelemetry();
        tx = new JulesRamTx(50_000, panelsTM, telemetry, "jules"); // ~16 min @ 50 Hz

        // --- start HTTP bridge with live streaming enabled ---
        buffer    = new JulesBuffer(4096);
        streamBus = new JulesStreamBus();
        JulesMetricsHttpAdapter adapter = new JulesMetricsHttpAdapter(buffer);

        // 🔒 Stable token: create once, then reuse forever
        Context ctx = hardwareMap.appContext; // FTC-provided Android Context
        String token = JulesTokenStore.getOrCreate(ctx);

        try {
            // pass the stable token so it doesn’t change each run
            http = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
            telemetry.addData("JULES", "HTTP up");
            panelsTM.debug(http.advertiseLine());
            panelsTM.debug("JULES Token", token);
            panelsTM.update(telemetry);
        } catch (IOException e) {
            telemetry.addData("JULES HTTP", "failed: %s", e.getMessage());
        }
    }

    @Override public void loop() {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        setDrivePower(power);
        tap.setLastCmd(power);

        double now = getRuntime();
        if (now - lastEmit >= PERIOD_S) {
            lastEmit = now;
            Metrics m = tap.sample(readHeadingDeg());

            // ✅ Put IP/token line FIRST so it always shows
            if (http != null) telemetry.addLine(http.advertiseLine());

            // 1) original pipeline: RAM + Panels
            tx.send(m);

            // 2) batch for /jules/dump
            buffer.push(m);

            // 3) live for /jules/stream
            streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
        }

        telemetry.update();
    }


    @Override public void stop() {
        try { tx.close(); } catch (Exception ignored) {}
        if (http != null) http.close();
        if (streamBus != null) streamBus.close();
    }

    private void setDrivePower(double p){ lf.setPower(p); rf.setPower(p); }
    private double readHeadingDeg(){
        // This will return the Yaw angle (Z-axis rotation) in degrees
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}