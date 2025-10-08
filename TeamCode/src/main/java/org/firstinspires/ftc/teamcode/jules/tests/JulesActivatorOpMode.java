// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/tests/JulesActivatorOpMode.java
package org.firstinspires.ftc.teamcode.jules.tests;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.jules.bridge.*;
import java.io.IOException;

@TeleOp(name = "JULES: Server Activator", group = "Jules")
public class JulesActivatorOpMode extends LinearOpMode {

    private JulesHttpBridge http;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Standard JULES Initialization ---
        JulesBuffer buffer = new JulesBuffer(4096);
        JulesStreamBus streamBus = new JulesStreamBus();
        JulesBufferJsonAdapter adapter = new JulesBufferJsonAdapter(buffer);
        Context ctx = hardwareMap.appContext;
        String token = JulesTokenStore.getOrCreate(ctx);

        try {
            http = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
        } catch (IOException e) {
            telemetry.addData("JULES FATAL", "Could not start HTTP bridge: " + e.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("✅ JULES Server Activated");
        telemetry.addLine(http.advertiseLine());
        telemetry.addLine("\nWaiting for client commands...");
        telemetry.update();

        waitForStart();

        // The OpMode is now running and waiting.
        // The main loop simply checks the command state.
        while (opModeIsActive() && !isStopRequested()) {

            switch (JulesCommand.getCommand()) {
                case MANUAL_DRIVE_TEST:
                    // In a future step, we'll add the code here to drive with gamepads
                    telemetry.addData("Executing Command", "MANUAL_DRIVE_TEST");
                    break;
                case FEEDFORWARD_TEST:
                    // In a future step, we'll add the automated feedforward test code here
                    telemetry.addData("Executing Command", "FEEDFORWARD_TEST");
                    break;
                case IDLE:
                default:
                    // Do nothing, just wait
                    telemetry.addData("Status", "IDLE - Waiting for command from client.");
                    break;
            }

            telemetry.update();
            sleep(100); // Slow down the loop while waiting for commands
        }

        // Cleanup
        if (http != null) http.close();
    }
}