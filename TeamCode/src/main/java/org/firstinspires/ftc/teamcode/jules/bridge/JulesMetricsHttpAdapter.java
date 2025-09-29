package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Adapts your JulesBuffer<Metrics> to the HTTP bridge.
 * - Treats Metrics.t as seconds since init.
 * - Emits JSON with:
 *   t  : milliseconds since init (long)
 *   ts : seconds since init (double, for charts)
 *   cmd: command power [-1..1]
 *   vel_ips, heading_deg, battery_v
 *   label (optional if Metrics.label != null)
 *
 * /dump?since=... uses the same units as `t` (milliseconds since init).
 */
public class JulesMetricsHttpAdapter implements JulesHttpBridge.Dumper, JulesHttpBridge.Labeler {

    private final JulesBuffer buffer;
    private final CopyOnWriteArrayList<String> labelsJson = new CopyOnWriteArrayList<>();

    public JulesMetricsHttpAdapter(JulesBuffer buffer) {
        this.buffer = buffer;
    }

    @Override
    public Iterator<String> dumpAll() {
        List<String> out = new ArrayList<>();
        Metrics[] snap = buffer.snapshot();
        for (Metrics m : snap) {
            if (m != null) out.add(encodePublic(m));
        }
        out.addAll(labelsJson);
        return out.iterator();
    }

    @Override
    public Iterator<String> dumpSince(long sinceMs) {
        List<String> out = new ArrayList<>();
        Metrics[] snap = buffer.snapshot();
        for (Metrics m : snap) {
            if (m == null) continue;
            long tMs = toMillis(m.t);
            if (tMs > sinceMs) out.add(encodePublic(m));
        }
        // include labels whose t > sinceMs (labels are already stored with ms)
        for (String lbl : labelsJson) {
            int i = lbl.indexOf("\"t\":");
            if (i >= 0) {
                int j = lbl.indexOf(',', i);
                String num = (j > i ? lbl.substring(i + 4, j) : lbl.substring(i + 4)).replaceAll("[^0-9]", "");
                try {
                    long t = Long.parseLong(num);
                    if (t > sinceMs) out.add(lbl);
                } catch (Exception ignore) {}
            }
        }
        return out.iterator();
    }

    @Override
    public void addLabel(long tMillis, String text) {
        // Keep labels in the same timebase as samples: milliseconds since init.
        String safe = (text == null) ? "" : text.replace("\\", "\\\\").replace("\"", "\\\"");
        labelsJson.add("{\"t\":" + tMillis + ",\"type\":\"label\",\"text\":\"" + safe + "\"}");
    }

    // ---- helpers ----

    private static long toMillis(double seconds) {
        return Math.round(seconds * 1000.0);
    }

    public static String encodePublic(Metrics m) {
        long tMs = toMillis(m.t);
        StringBuilder sb = new StringBuilder(128);
        sb.append("{\"t\":").append(tMs)
                .append(",\"ts\":").append(m.t)               // seconds for charting
                .append(",\"cmd\":").append(m.cmdPower)
                .append(",\"vel_ips\":").append(m.velIPS)
                .append(",\"heading_deg\":").append(m.headingDeg)
                .append(",\"battery_v\":").append(m.batteryV);
        if (m.label != null) {
            String safe = m.label.replace("\\", "\\\\").replace("\"", "\\\"");
            sb.append(",\"label\":\"").append(safe).append("\"");
        }
        sb.append("}");
        return sb.toString();
    }
}
