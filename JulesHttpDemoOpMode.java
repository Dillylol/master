package org.firstinspires.ftc.teamcode.jules.bridge;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

/**
 * Demo OpMode to prove the bridge works.
 * Replace TODOs to connect to your real buffer/sample model.
 */
@TeleOp(name = "JULES HTTP Demo (NanoHTTPD)", group = "JULES")
public class JulesHttpDemoOpMode extends OpMode {

    private JulesHttpBridge http;

    @Override public void init() {
        // === TODO: Replace this with your real buffer snapshot ===
        // If your buffer already stores JSON lines, use dumperFrom(...)
        boolean bufferAlreadyJson = false; // flip true if you have jsonLinesIterator()

        if (bufferAlreadyJson) {
            // Example if your buffer exposes an iterator of JSON lines directly:
            Iterator<String> it = Collections.<String>emptyList().iterator(); // replace with buffer.jsonLinesIterator()
            JulesHttpBridge.Dumper dumper = JulesHttpBridge.dumperFrom(() -> it);
            JulesHttpBridge.Labeler labeler = (t, text) -> { /* buffer.addLabel(t, text); */ };
            try {
                http = new JulesHttpBridge(58080, dumper, labeler);
                telemetry.addData("JULES", "HTTP up");
            } catch (IOException e) {
                telemetry.addData("JULES", "HTTP failed: %s", e.getMessage());
            }
            return;
        }

        // === Generic adapter path (works with any sample struct) ===
        // Define your sample type (replace with your real one)
        class Sample { long t; double vel; double cmd; double dt = Double.NaN; }

        JulesBufferJsonAdapter<Sample> adapter = new JulesBufferJsonAdapter<>(
                // SnapshotProvider<T>
                new JulesBufferJsonAdapter.SnapshotProvider<Sample>() {
                    @Override public List<Sample> snapshotAll() {
                        // TODO: Replace with a stable snapshot from your buffer
                        // This placeholder returns an empty immutable list.
                        return Collections.unmodifiableList(new ArrayList<Sample>());
                    }
                },
                // tsExtractor
                (Sample s) -> s.t,
                // jsonEncoder (no trailing newline)
                (Sample s) -> {
                    // TODO: Map your real fields here; below is a minimal schema
                    StringBuilder sb = new StringBuilder();
                    sb.append("{\"t\":").append(s.t)
                            .append(",\"vel\":").append(s.vel)
                            .append(",\"cmd\":").append(s.cmd);
                    if (!Double.isNaN(s.dt)) sb.append(",\"dt\":").append(s.dt);
                    sb.append("}");
                    return sb.toString();
                }
        );

        try {
            http = new JulesHttpBridge(58080, adapter, adapter); // dumper + labeler
            telemetry.addData("JULES", "HTTP up");
        } catch (IOException e) {
            telemetry.addData("JULES", "HTTP failed: %s", e.getMessage());
        }
    }

    @Override public void loop() {
        if (http != null) telemetry.addLine(http.advertiseLine());
        // Optional: add quick labels from the DS for testing
        // if (gamepad1.x) { adapter.addLabel(System.currentTimeMillis(), "TestLabel"); }
    }

    @Override public void stop() {
        if (http != null) http.close();
    }
}
