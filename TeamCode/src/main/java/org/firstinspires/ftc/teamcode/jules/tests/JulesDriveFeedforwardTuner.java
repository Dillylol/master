// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/tests/JulesDriveFeedforwardTuner.java
package org.firstinspires.ftc.teamcode.jules.tests;

import android.content.Context;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBufferJsonAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // For wheel radius, etc.

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "JULES: Drive Feedforward Tuner", group = "Jules")
public class JulesDriveFeedforwardTuner extends LinearOpMode {

    public static double TEST_VOLTAGE = 0.7; // Run motors at 70% power
    public static double TEST_DURATION_S = 5.0; // Run test for 5 seconds

    private JulesHttpBridge http;
    private JulesStreamBus streamBus;
    private JulesRamTx tx;
    private JulesTap tap;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Standard JULES Initialization ---
        TelemetryManager panelsTM = PanelsTelemetry.INSTANCE.getTelemetry();
        tx = new JulesRamTx(50_000, panelsTM, telemetry, "jules");

        JulesBuffer buffer = new JulesBuffer(4096);
        streamBus = new JulesStreamBus();
        JulesBufferJsonAdapter bufferAdapter = new JulesBufferJsonAdapter(buffer);
        JulesMetricsHttpAdapter metricsAdapter = new JulesMetricsHttpAdapter(buffer);
        Context ctx = hardwareMap.appContext;
        String token = JulesTokenStore.getOrCreate(ctx);

        try {
            http = new JulesHttpBridge(58080, buffer, bufferAdapter, token, streamBus);
        } catch (IOException e) {
            telemetry.addData("JULES HTTP", "failed: %s", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // --- Hardware Initialization ---
        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();
        List<DcMotorEx> motors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "rightRear")
        );
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Assuming your right-side motors are reversed
        motors.get(2).setDirection(DcMotorEx.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorEx.Direction.REVERSE);

        // JulesTap for velocity measurement. Ensure these constants match your robot.
        tap = new JulesTap(537.7, Constants.WHEEL_RADIUS * 2, Constants.GEAR_RATIO, vs, motors);

        telemetry.addLine("JULES Drive Feedforward Tuner");
        telemetry.addLine("Ready to Start Test.");
        telemetry.addLine();
        telemetry.addLine(http.advertiseLine());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addLine("Test in progress...");
        telemetry.update();

        ElapsedTime testTimer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz

        // Main test loop
        while (opModeIsActive() && testTimer.seconds() < TEST_DURATION_S) {
            double now = getRuntime();
            // Set motor power
            for (DcMotorEx motor : motors) {
                motor.setPower(TEST_VOLTAGE);
            }
            tap.setLastCmd(TEST_VOLTAGE);

            // Stream data at 50Hz
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                Metrics m = tap.sample(0); // Heading is not critical for this test
                tx.send(m);
                buffer.push(m);
                streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
            }
        }

        // Stop motors and clean up
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }

        telemetry.clear();
        telemetry.addLine("Test Complete.");
        telemetry.update();

        // Keep server alive for a bit to allow client to dump data
        sleep(10000);

        if (http != null) http.close();
        if (streamBus != null) streamBus.close();
    }
}