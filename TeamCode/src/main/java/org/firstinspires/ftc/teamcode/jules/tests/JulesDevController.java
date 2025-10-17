package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Dev controller that consumes the persistent JULES bridge (managed by JulesBridgeManager)
 * and executes simple motion commands coming from the client.
 *
 * Expected commands (case-insensitive):
 *  - DRIVE_FORWARD_<seconds>T_<power>P
 *  - DRIVE_BACKWARD_<seconds>T_<power>P
 *  - STRAFE_LEFT_<seconds>T_<power>P
 *  - STRAFE_RIGHT_<seconds>T_<power>P
 *  - TURN_LEFT_<seconds>T_<power>P
 *  - TURN_RIGHT_<seconds>T_<power>P
 *  - STOP
 */
@TeleOp(name = "JULES: Dev Controller", group = "Jules")
public class JulesDevController extends LinearOpMode {

    // --- Robot constants (tune for your build) ---
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // 96mm goBILDA mecanum
    private static final double GEAR_RATIO = 1.0;

    private static final List<String> COMMAND_REFERENCE = Arrays.asList(
            "DRIVE_FORWARD_<seconds>T_<power>P",
            "DRIVE_BACKWARD_<seconds>T_<power>P",
            "STRAFE_LEFT_<seconds>T_<power>P",
            "STRAFE_RIGHT_<seconds>T_<power>P",
            "TURN_LEFT_<seconds>T_<power>P",
            "TURN_RIGHT_<seconds>T_<power>P",
            "STOP"
    );

    // JULES bridge accessors
    private JulesBridgeManager bridgeManager;

    // Robot / sensing
    private VoltageSensor vs;
    private IMU imu;
    private Follower follower;
    private JulesTap tap;

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        setupJules();

        var status = bridgeManager.getStatusSnapshot();
        telemetry.addLine("✅ JULES Dev Controller Initialized");
        telemetry.addLine(status != null ? status.advertiseLine : "JULES: stopped");
        telemetry.addLine("\nReady to receive client commands.");
        telemetry.addLine("Supported commands:");
        for (String cmd : COMMAND_REFERENCE) telemetry.addLine(" • " + cmd);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Keep odometry fresh
            follower.update();

            // Refresh stream bus in case bridge was toggled
            JulesStreamBus bus = (bridgeManager != null) ? bridgeManager.getStreamBus() : null;

            // Pull latest command (exactly-once semantics handled by JulesCommand)
            String raw = JulesCommand.getAndClearCommand();
            if (raw != null) {
                telemetry.addData("Received", raw);
                telemetry.update();

                parseAndExecute(raw, bus);

                // Ensure stop after each discrete command
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                telemetry.addData("Status", "IDLE - Waiting for command.");
                telemetry.update();
            }

            sleep(50); // poll ~20 Hz
        }
    }

    /** Parse command string and execute. */
    private void parseAndExecute(String commandRaw, JulesStreamBus bus) {
        if (commandRaw == null) return;
        String command = commandRaw.trim();
        if (command.isEmpty()) return;

        String upper = command.toUpperCase(Locale.US);
        double duration = Math.max(0, parseValue(upper, "T", 1.0));
        double power = clampPower(parseValue(upper, "P", 0.5));
        double mag = Math.abs(power);

        if (upper.startsWith("DRIVE_FORWARD")) {
            executeMovement(duration, mag, 0, 0, command, bus);
        } else if (upper.startsWith("DRIVE_BACKWARD")) {
            executeMovement(duration, -mag, 0, 0, command, bus);
        } else if (upper.startsWith("STRAFE_LEFT")) {
            executeMovement(duration, 0, -mag, 0, command, bus);
        } else if (upper.startsWith("STRAFE_RIGHT")) {
            executeMovement(duration, 0, mag, 0, command, bus);
        } else if (upper.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -mag, command, bus);
        } else if (upper.startsWith("TURN_RIGHT")) {
            executeMovement(duration, 0, 0, mag, command, bus);
        } else if (upper.startsWith("STOP")) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.addData("Command", "STOP");
            telemetry.update();
        } else {
            telemetry.addData("Unknown", command);
            telemetry.update();
        }
    }

    /**
     * Executes a teleop-style movement while streaming metrics out the JULES stream bus.
     */
    private void executeMovement(double durationSec,
                                 double y, double x, double r,
                                 String label,
                                 JulesStreamBus bus) {
        ElapsedTime timer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz stream

        boolean startLabelSent = false;
        if (bus != null) {
            bus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
            startLabelSent = true;
        }

        while (opModeIsActive() && timer.seconds() < durationSec) {
            follower.update();
            follower.setTeleOpDrive(y, x, r, true);

            // Update last commanded power for visibility
            double commandPower = (y != 0) ? y : (x != 0 ? x : r);
            if (tap != null) tap.setLastCmd(commandPower);

            double now = getRuntime();
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                Metrics m = tap != null ? tap.sample(imu, follower) : null;
                JulesStreamBus currentBus = (bus != null) ? bus : (bridgeManager != null ? bridgeManager.getStreamBus() : null);
                if (currentBus != null) {
                    if (!startLabelSent) {
                        currentBus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
                        startLabelSent = true;
                    }
                    if (m != null) currentBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
                    bus = currentBus; // keep latest reference
                }
            }

            telemetry.addData("Executing", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), durationSec);
            telemetry.update();
        }

        JulesStreamBus endBus = (bus != null) ? bus : (bridgeManager != null ? bridgeManager.getStreamBus() : null);
        if (startLabelSent && endBus != null) {
            endBus.publishJsonLine(String.format("{\"label\":\"END: %s\"}", label));
        }
    }

    // Convenience overload for legacy 5‑arg call sites (defaults to current bus)
    @SuppressWarnings("unused")
    private void executeMovement(double durationSec, double y, double x, double r, String label) {
        JulesStreamBus bus = (bridgeManager != null) ? bridgeManager.getStreamBus() : null;
        executeMovement(durationSec, y, x, r, label, bus);
    }

    // --- Helpers ---
    private double parseValue(String command, String id, double defaultValue) {
        Pattern p = Pattern.compile("([-+]?\\d*\\.?\\d+)" + id);
        Matcher m = p.matcher(command);
        if (m.find()) {
            try { return Double.parseDouble(m.group(1)); }
            catch (NumberFormatException ignored) { return defaultValue; }
        }
        return defaultValue;
    }

    private double clampPower(double p) {
        if (Double.isNaN(p)) return 0;
        return Math.max(-1.0, Math.min(1.0, p));
    }

    private void setupJules() {
        bridgeManager = JulesBridgeManager.getInstance();
        // If your manager needs appContext, expose a prepare(context) and call it here.
        // bridgeManager.prepare(hardwareMap.appContext);
    }

    private void setupHardware() {
        follower = Constants.createFollower(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        vs = hardwareMap.voltageSensor.iterator().next();

        List<DcMotorEx> motors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "lf"),
                hardwareMap.get(DcMotorEx.class, "rf"),
                hardwareMap.get(DcMotorEx.class, "lr"),
                hardwareMap.get(DcMotorEx.class, "rr")
        );

        tap = new JulesTap(
                537.7,                 // ticks per rev (example)
                WHEEL_DIAMETER_INCHES,
                GEAR_RATIO,
                vs,
                motors
        );
    }
}
