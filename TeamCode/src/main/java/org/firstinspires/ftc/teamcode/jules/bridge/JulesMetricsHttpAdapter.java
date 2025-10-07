// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesMetricsHttpAdapter.java

package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.teamcode.jules.Metrics;
import com.google.gson.Gson;

public class JulesMetricsHttpAdapter {
    private final JulesBuffer buffer;

    public JulesMetricsHttpAdapter(JulesBuffer buffer) {
        this.buffer = buffer;
    }

    public String dumpJsonLines() {
        StringBuilder sb = new StringBuilder();
        Metrics[] snapshot = buffer.snapshot();
        for (Metrics m : snapshot) {
            if (m != null) {
                sb.append(encodePublic(m));
                sb.append('\n');
            }
        }
        return sb.toString();
    }

    public static String encodePublic(Metrics m) {
        if (m == null) return "{}";

        // Use a StringBuilder for efficient string construction
        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"ts\":").append(m.t)
                .append(",\"cmd\":").append(m.cmdPower)
                .append(",\"vel_ips\":").append(m.velIPS)
                .append(",\"battery_v\":").append(m.batteryV)
                .append(",\"x\":").append(m.x)
                .append(",\"y\":").append(m.y)
                .append(",\"heading\":").append(m.heading)
                .append(",\"heading_deg\":").append(m.headingDeg)
                .append(",\"pitch\":").append(m.pitch)
                .append(",\"roll\":").append(m.roll)
                .append(",\"yawRate\":").append(m.yawRate)
                .append(",\"pitchRate\":").append(m.pitchRate)
                .append(",\"rollRate\":").append(m.rollRate);

        // --- NEW: Correctly handle the label field ---
        // If a label exists, escape it and add it to the JSON object.
        if (m.label != null && !m.label.isEmpty()) {
            String safeLabel = m.label.replace("\\", "\\\\").replace("\"", "\\\"");
            sb.append(",\"label\":\"").append(safeLabel).append("\"");
        }
        // ---------------------------------------------

        sb.append("}");
        return sb.toString();
    }
}