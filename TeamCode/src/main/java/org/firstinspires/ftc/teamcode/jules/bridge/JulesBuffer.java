package org.firstinspires.ftc.teamcode.jules.bridge;
import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.util.concurrent.atomic.AtomicInteger;

public class JulesBuffer {
    private final Metrics[] buf;
    private final AtomicInteger writeIdx = new AtomicInteger(0);
    private volatile int count = 0;

    public JulesBuffer(int capacity) { this.buf = new Metrics[capacity]; }

    public void push(Metrics m) {
        int i = writeIdx.getAndIncrement();
        if (i >= buf.length) {
            writeIdx.set(1);
            i = 0;
        }
        buf[i] = m;
        if (count < buf.length) count++;
    }

    /** Returns a copy oldest→newest. */
    public Metrics[] snapshot() {
        Metrics[] out = new Metrics[count];
        int w = writeIdx.get() % buf.length;
        int n = count;
        for (int k = 0; k < n; k++) {
            int idx = (w - n + k);
            if (idx < 0) idx += buf.length;
            out[k] = buf[idx];
        }
        return out;
    }

    public void clear() { writeIdx.set(0); count = 0; }
    public int size() { return count; }
    public int capacity() { return buf.length; }
}
