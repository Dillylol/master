package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.JulesService;
import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBufferJsonAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.jules.JulesTap;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@TeleOp(name = "JULES: Dev Controller", group = "Jules")
public class JulesDevController extends LinearOpMode {

    // --- Constants for physical properties of the robot ---
    // TODO: Tune these values for your specific robot
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // For standard 96mm goBILDA mecanum wheels
    private static final double GEAR_RATIO = 1.0;            // Assuming no external gearing on the drivetrain

    // JULES Components
    private JulesHttpBridge http;
    private JulesStreamBus streamBus;
    private JulesTap tap;

    // Robot Hardware & Control
    private VoltageSensor vs;
    private IMU imu;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and JULES components
        setupHardware();
        setupJules();

        telemetry.addLine("✅ JULES Dev Controller Initialized");
        if (http != null) {
            telemetry.addLine(http.advertiseLine());
        }
        telemetry.addLine("\nReady to receive client commands.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // The OpMode is now running and waiting for commands.
        while (opModeIsActive() && !isStopRequested()) {
            // CRITICAL: follower.update() must be called every loop to update odometry
            follower.update();

            // Get the latest command from the client. This also clears it.
            String command = JulesCommand.getAndClearCommand();

            if (command != null) {
                telemetry.addData("Received Command", command);
                telemetry.update();

                parseAndExecute(command); // Execute the command

                // Ensure the robot is stopped after the command finishes
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                telemetry.addData("Status", "IDLE - Waiting for command.");
                telemetry.update();
            }

            sleep(50); // Poll for commands at 20Hz
        }

        // Cleanup
        if (http != null) http.close();
    }

    /**
     * Parses the command string and calls the appropriate robot action.
     * @param command The command string from the client (e.g., "DRIVE_FORWARD_1.5T_0.5V")
     */
    private void parseAndExecute(String command) {
        // Default values for movement
        double duration = parseValue(command, "T", 1.0); // Default to 1 second
        double power = parseValue(command, "V", 0.4);    // Default to 0.4 power

        if (command.startsWith("DRIVE_FORWARD")) {
            executeMovement(duration, power, 0, 0, command);
        } else if (command.startsWith("DRIVE_BACKWARD")) {
            executeMovement(duration, -power, 0, 0, command);
        } else if (command.startsWith("STRAFE_LEFT")) {
            executeMovement(duration, 0, -power, 0, command);
        } else if (command.startsWith("STRAFE_RIGHT")) {
            executeMovement(duration, 0, power, 0, command);
        } else if (command.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -power, command);
        } else if (command.startsWith("TURN_RIGHT")) {
            executeMovement(duration, 0, 0, power, command);
        }
    }

    /**
     * Executes a teleop-style movement for a specified duration while streaming data.
     */
    private void executeMovement(double duration, double y, double x, double r, String label) {
        ElapsedTime timer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz data streaming rate

        // Add a label to the data stream to mark the start of the test
        streamBus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));

        while (opModeIsActive() && timer.seconds() < duration) {
            // Keep odometry and robot state updated
            follower.update();

            // Command the robot using Pedro Pathing's teleop drive
            follower.setTeleOpDrive(y, x, r, true);

            // Set the command power for logging purposes
            double commandPower = y != 0 ? y : (x != 0 ? x : r);
            tap.setLastCmd(commandPower);

            // Stream data at the specified frequency (50Hz)
            double now = getRuntime();
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                Metrics m = tap.sample(imu, follower);
                streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
            }

            telemetry.addData("Executing", "%s", label);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), duration);
            telemetry.update();
        }

        // Add a label to mark the end of the test
        streamBus.publishJsonLine(String.format("{\"label\":\"END: %s\"}", label));
    }

    /**
     * A helper to parse numeric values from a command string.
     */
    private double parseValue(String command, String id, double defaultValue) {
        Pattern p = Pattern.compile("(\\d*\\.?\\d+)" + id);
        Matcher m = p.matcher(command);
        if (m.find()) {
            try {
                return Double.parseDouble(m.group(1));
            } catch (NumberFormatException e) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    // --- Helper methods for initialization ---

    private void setupJules() {
        JulesBuffer buffer = new JulesBuffer(8192);
        streamBus = new JulesStreamBus();
        JulesBufferJsonAdapter adapter = new JulesBufferJsonAdapter(buffer);
        String token = JulesTokenStore.getOrCreate(hardwareMap.appContext);

        try {
            http = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
        } catch (IOException e) {
            telemetry.addData("JULES FATAL", "Could not start HTTP bridge: " + e.getMessage());
            http = null;
        }
    }

    private void setupHardware() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);

        // Get hardware references
        imu = hardwareMap.get(IMU.class, "imu");
        vs = hardwareMap.voltageSensor.iterator().next();

        // Create the JULES data tap with your robot's specific motors and constants
        List<DcMotorEx> motors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "lf"),
                hardwareMap.get(DcMotorEx.class, "rf"),
                hardwareMap.get(DcMotorEx.class, "lr"),
                hardwareMap.get(DcMotorEx.class, "rr")
        );

        tap = new JulesTap(
                537.7, // Ticks Per Revolution for your motors (e.g., goBILDA 5203 series)
                WHEEL_DIAMETER_INCHES,
                GEAR_RATIO,
                vs,
                motors);
    }
}