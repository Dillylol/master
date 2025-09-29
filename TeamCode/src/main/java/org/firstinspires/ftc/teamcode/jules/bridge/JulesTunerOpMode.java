package org.firstinspires.ftc.teamcode.jules.bridge;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name = "JULES Tuner (HTTP)", group = "JULES")
public class JulesTunerOpMode extends OpMode {

    private JulesBuffer buffer;       // your ring buffer of Metrics
    private JulesHttpBridge http;     // NanoHTTPD bridge

    @Override public void init() {
        // Size as you like; your tap code should push Metrics into this buffer.
        buffer = new JulesBuffer(4096);

        // TODO: wire your JulesTap/JulesRamTx so they call buffer.push(new Metrics(...)) regularly.

        // Adapter that turns Metrics -> JSONL for the bridge
        JulesMetricsHttpAdapter adapter = new JulesMetricsHttpAdapter(buffer);

        try {
            http = new JulesHttpBridge(58080, adapter, adapter); // dumper + labeler
            telemetry.addData("JULES", "HTTP up");
        } catch (IOException e) {
            telemetry.addData("JULES", "HTTP failed: %s", e.getMessage());
        }
    }

    @Override public void loop() {
        if (http != null) telemetry.addLine(http.advertiseLine()); // shows http://IP:58080 token=...
        telemetry.update();
    }

    @Override public void stop() {
        if (http != null) http.close();
    }
}
