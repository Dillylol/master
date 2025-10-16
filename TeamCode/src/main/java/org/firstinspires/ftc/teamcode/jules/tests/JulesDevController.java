// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/JulesDevController.java
package org.firstinspires.ftc.teamcode.jules.tests;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@TeleOp(name = "JULES: Dev Controller", group = "Jules")
public class JulesDevController extends LinearOpMode {

    // JULES Components
    private JulesHttpBridge http;
    private JulesStreamBus streamBus;
    private JulesTap tap;

    // Robot Hardware & Control
    private List<DcMotorEx> motors;
    private VoltageSensor vs;
    private IMU imu;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        setupJules();
        setupHardware();

        telemetry.addLine("✅ JULES Dev Controller Initialized");
        if (http != null) telemetry.addLine(http.advertiseLine());
        telemetry.addLine("\nReady to receive client commands.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // CRITICAL: follower.update() must be called every loop to update odometry
            follower.update();

            // Get the latest command from the client. This also clears it.
            String command = JulesCommand.getAndClearCommand();

            if (command != null) {
                telemetry.addData("Received Command", command);
                telemetry.update();
                parseAndExecute(command); // Execute the command
                // After execution, stop the robot
                follower.setTeleOpDrive(0,0,0);
            } else {
                telemetry.addData("Status", "IDLE - Waiting for command.");
                telemetry.update();
            }

            sleep(50); // Poll for commands at 20Hz
        }

        if (http != null) http.close();
    }

    /**
     * Parses the command string and calls the appropriate robot action.
     * @param command The command string from the client (e.g., "DRIVE_FORWARD_4T_0.5V")
     */
    private void parseAndExecute(String command) {
        double duration = parseValue(command, "T", 2.0); // Default 2s
        double power = parseValue(command, "V", 0.5);  // Default 0.5 power

        if (command.startsWith("DRIVE_FORWARD")) {
            executeMovement(duration, power, 0, 0);
        } else if (command.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -power); // Negative rotational power
        }
        // Add more 'else if' blocks here for STRAFE, TURN_RIGHT, etc.
    }

    /**
     * Executes a teleop-style movement for a specified duration while streaming data.
     */
    private void executeMovement(double duration, double y, double x, double r) {
        ElapsedTime timer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz

        while (opModeIsActive() && timer.seconds() < duration) {
            // Keep constants and odometry live
            follower.update();
            tap.updateConstants(Constants.WHEEL_RADIUS * 2, Constants.GEAR_RATIO);

            // Use Pedro Pathing to command the robot
            follower.setTeleOpDrive(y, x, r);
            tap.setLastCmd(y != 0 ? y : (x != 0 ? x : r));

            // Stream data at 50Hz
            double now = getRuntime();
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                Metrics m = tap.sample(imu, follower);
                streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
            }

            telemetry.addData("Executing", "y: %.2f, x: %.2f, r: %.2f", y, x, r);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), duration);
            telemetry.update();
        }
    }

    private double parseValue(String command, String id, double defaultValue) {
        Pattern p = Pattern.compile("(\\d*\\.?\\d+)" + id);
        Matcher m = p.matcher(command);
        if (m.find()) {
            try { return Double.parseDouble(m.group(1)); }
            catch (NumberFormatException e) { return defaultValue; }
        }
        return defaultValue;
    }

    // --- Helper methods for initialization ---
    private void setupJules() { /* ... unchanged ... */ }
    private void setupHardware() { /* ... unchanged, but ensure follower is created ... */ }
}