package org.firstinspires.ftc.teamcode.jules.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.jules.FileTx;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.io.IOException;

@TeleOp(name="Jules Tap Demo", group="Jules")
public class JulesTapDemo extends OpMode {
    // Map the same motors Panels uses for drive
    DcMotorEx leftFront, rightFront;
    BNO055IMU imu;
    VoltageSensor vs;

    // Fill these from your known hardware OR let Jules v1 generate a Setup mode first
    static final double TPR  = 537.7; // ticks per rev (e.g., goBILDA 312 RPM)
    static final double W_IN = 3.78;  // wheel diameter in inches
    static final double GR   = 1.0;   // external gear ratio

    JulesTap tap;
    FileTx tx;

    @Override public void init() {
        leftFront  = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        vs = hardwareMap.voltageSensor.iterator().next();

        // IMU init omitted for brevity; use whatever your quickstart uses
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        tap = new JulesTap(TPR, W_IN, GR, vs, leftFront, rightFront);
        try { tx = new FileTx(); } catch (IOException e) {
            telemetry.addLine("JULES FileTx failed to open"); telemetry.update();
        }
    }

    @Override public void loop() {
        // Example: get desired power from your Panels UI, or gamepad for now
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        setDrivePower(power);                  // <-- your drive method
        tap.setLastCmd(power);                 // 1) record the command power

        double headingDeg = getHeadingDeg();   // 2) read IMU like Panels does
        Metrics m = tap.sample(headingDeg);    // 3) sample the frame

        if (tx != null) tx.send(m);            // 4) write JSONL line (V1 downlink)

        // Optional: mirror to Panels-style telemetry keys so it looks identical
        telemetry.addData("drive/velIPS", m.velIPS);
        telemetry.addData("command/power", m.cmdPower);
        telemetry.addData("drive/headingDeg", m.headingDeg);
        telemetry.addData("battery/V", m.batteryV);
        telemetry.update();
    }

    @Override public void stop() {
        try { if (tx != null) tx.close(); } catch (Exception ignored) {}
    }

    private void setDrivePower(double p) {
        leftFront.setPower(p);
        rightFront.setPower(p);
        // add other wheels if mecanum; this demo keeps it short
    }
    private double getHeadingDeg() {
        // Replace with your quickstart’s IMU helper. Example if using BNO055:
        return imu.getAngularOrientation().firstAngle; // depends on axes order; match Panels
    }
}
