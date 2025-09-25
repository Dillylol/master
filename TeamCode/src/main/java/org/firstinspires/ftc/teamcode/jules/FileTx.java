package org.firstinspires.ftc.teamcode.jules;

import java.io.*;

public class FileTx implements AutoCloseable {
    private final Writer out;

    public FileTx() throws IOException {
        File dir = new File("/sdcard/FIRST/jules");
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        File file = new File(dir, "session_" + System.currentTimeMillis() + ".jsonl");
        out = new BufferedWriter(new FileWriter(file));
    }

    /** Write one telemetry frame as JSONL (one JSON object per line). */
    public synchronized void send(Metrics m) {
        try {
            out.write(String.format(
                    "{\"t\":%.4f,\"cmdPower\":%.3f,\"velIPS\":%.3f,\"headingDeg\":%.2f,\"batteryV\":%.2f%s}\n",
                    m.t, m.cmdPower, m.velIPS, m.headingDeg, m.batteryV,
                    (m.label != null ? ",\"label\":\"" + escape(m.label) + "\"" : "")
            ));
        } catch (IOException ignored) {}
    }

    public synchronized void label(String text){
        Metrics m = new Metrics();
        m.t = 0; m.label = text;
        send(m);
    }

    @Override public synchronized void close() {
        try { out.flush(); out.close(); } catch (IOException ignored) {}
    }

    private static String escape(String s){ return s.replace("\"","\\\""); }
}
