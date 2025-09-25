// org.firstinspires.ftc.teamcode.jules.JulesRamTx
package org.firstinspires.ftc.teamcode.jules;

import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JulesRamTx implements AutoCloseable {
    private final JulesBuffer buffer;
    private final TelemetryManager panelsTel;   // Panels bus
    private final Telemetry dsTelemetry;        // DS telemetry (for flush)
    private final String prefix;

    public JulesRamTx(int capacity, TelemetryManager panelsTelemetry, Telemetry dsTelemetry, String topicPrefix) {
        this.buffer = new JulesBuffer(capacity);
        this.panelsTel = panelsTelemetry;
        this.dsTelemetry = dsTelemetry;
        this.prefix = topicPrefix != null ? topicPrefix : "jules";
    }

    public void send(Metrics m) {
        buffer.push(m);
        panelsTel.debug(
                prefix + "/t: "        + String.format("%.3f", m.t),
                prefix + "/cmd: "      + String.format("%.2f", m.cmdPower),
                prefix + "/velIPS: "   + String.format("%.2f", m.velIPS),
                prefix + "/heading: "  + String.format("%.1f", m.headingDeg),
                prefix + "/batteryV: " + String.format("%.2f", m.batteryV)
        );
        panelsTel.update(dsTelemetry); // flush to Panels + DS
    }

    public void label(String text) {
        Metrics mark = new Metrics(); mark.t = System.nanoTime()*1e-9; mark.label = text;
        buffer.push(mark);
        panelsTel.debug(prefix + "/label: " + text);
        panelsTel.update(dsTelemetry);
    }

    public Metrics[] snapshot(){ return buffer.snapshot(); }
    public int size(){ return buffer.size(); }
    public int capacity(){ return buffer.capacity(); }
    @Override public void close() {}
}