package org.firstinspires.ftc.teamcode.jules.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Panels-style toggle OpMode that starts/stops the persistent JULES bridge
 * and mirrors status to BOTH the FTC Driver Station telemetry and the Panels UI telemetry.
 *
 * Panels will display the **full (unmasked) token** for fast pairing.
 * Driver Station will continue to display the masked token.
 */
@TeleOp(name = "JULES: Bridge Switch", group = "Jules")
public class JulesBridgeSwitch extends OpMode {

    private JulesBridgeManager manager;
    private String preparedIp;
    private String preparedToken;

    // Panels telemetry (separate from FTC DS telemetry)
    private Telemetry panelsTl;

    @Override
    public void init() {
        manager = JulesBridgeManager.getInstance();
        manager.prepare(hardwareMap.appContext);
        preparedIp = manager.defaultIp();
        preparedToken = manager.ensureToken(hardwareMap.appContext);

        // Hook Panels telemetry using the same API as your BotelloDATA program
        panelsTl = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        // DS shows masked; Panels shows unmasked
        telemetry.addLine("Press START to enable JULES");
        telemetry.addData("IP", preparedIp);
        telemetry.addData("Token", JulesBridgeManager.maskToken(preparedToken));
        telemetry.update();

        panelsTl.addLine("Press START to enable JULES");
        panelsTl.addData("IP", preparedIp);
        panelsTl.addData("Token", preparedToken); // UNMASKED on Panels
        panelsTl.update();
    }

    @Override
    public void start() {
        manager.start(preparedIp, preparedToken);
        addLine("Starting JULES bridge…");
        update();
    }

    @Override
    public void loop() {
        JulesBridgeManager.Status status = manager.getStatusSnapshot();

        if (status == null) {
            addData("Status", "STOPPED");
            addLine("Press START to enable JULES");
            update();
            return;
        }

        // Driver Station (masked)
        telemetry.addData("Status", status.running ? "RUNNING" : "STOPPED");
        telemetry.addData("IP", nullSafe(status.ip));
        telemetry.addData("Port", status.port);
        telemetry.addData("Token", JulesBridgeManager.maskToken(status.token));
        telemetry.addData("Uptime", formatDuration(status.uptimeMs));
        telemetry.addData("Retries", status.retryCount);
        if (status.lastError != null && !status.lastError.isEmpty()) {
            telemetry.addData("Last error", status.lastError);
        }
        telemetry.addLine(nullSafe(status.advertiseLine));
        telemetry.update();

        // Panels (UNMASKED)
        if (panelsTl != null) {
            panelsTl.addData("Status", status.running ? "RUNNING" : "STOPPED");
            panelsTl.addData("IP", nullSafe(status.ip));
            panelsTl.addData("Port", status.port);
            panelsTl.addData("Token", nullSafe(status.token)); // full token
            panelsTl.addData("Uptime", formatDuration(status.uptimeMs));
            panelsTl.addData("Retries", status.retryCount);
            if (status.lastError != null && !status.lastError.isEmpty()) {
                panelsTl.addData("Last error", status.lastError);
            }
            panelsTl.addLine(nullSafe(status.advertiseLine));
            if (!status.running) panelsTl.addLine("Press START to enable JULES");
            panelsTl.update();
        }
    }

    @Override
    public void stop() {
        manager.stop();
        addLine("JULES bridge stopped");
        update();
    }

    // -------- Telemetry helpers: add to BOTH where appropriate --------
    private void addData(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsTl != null) panelsTl.addData(caption, value);
    }

    private void addLine(String line) {
        telemetry.addLine(line);
        if (panelsTl != null) panelsTl.addLine(line);
    }

    private void update() {
        telemetry.update();
        if (panelsTl != null) panelsTl.update();
    }

    private static String nullSafe(String s) { return s == null ? "-" : s; }

    private String formatDuration(long ms) {
        long totalSeconds = ms / 1000L;
        long hours = totalSeconds / 3600L;
        long minutes = (totalSeconds % 3600L) / 60L;
        long seconds = totalSeconds % 60L;
        if (hours > 0) return String.format("%dh %02dm %02ds", hours, minutes, seconds);
        if (minutes > 0) return String.format("%dm %02ds", minutes, seconds);
        return String.format("%ds", seconds);
    }
}