
package org.firstinspires.ftc.teamcode.jules.bridge;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Function;

/**
 * Minimal, non-invasive adapter that turns your existing JulesBuffer into
 * JSONL for the HTTP bridge, with optional incremental (since=) support
 * and label injection even if your buffer doesn't natively store labels.
 *
 * You supply:
 *  1) A way to snapshot-read samples as a stable {@code List<T>} or {@code Iterator<T>}.
 *  2) A JSON encoder for each sample (T -> String JSON).
 *  3) A timestamp extractor (T -> epochMs) so we can support since-filtering.
 *
 * If your buffer already stores JSON strings per sample, you can skip this and use
 * {@code JulesHttpBridge.dumperFrom(() -> buffer.jsonLinesIterator())} instead.
 */
public class JulesBufferJsonAdapter<T> implements JulesHttpBridge.Dumper, JulesHttpBridge.Labeler {

    /** Your immutable snapshot of samples (replace with a call into JulesBuffer). */
    public interface SnapshotProvider<T> {
        /** Return a stable, read-only view of all samples collected so far. */
        List<T> snapshotAll();
    }

    private final SnapshotProvider<T> provider;
    private final Function<T, Long> tsExtractor;   // T -> epochMs
    private final Function<T, String> jsonEncoder; // T -> JSON (no trailing newline)

    // Sidecar labels (JSONL lines) for teams whose buffer doesn't yet store labels.
    private final CopyOnWriteArrayList<Label> labels = new CopyOnWriteArrayList<>();

    public JulesBufferJsonAdapter(SnapshotProvider<T> provider,
                                  Function<T, Long> tsExtractor,
                                  Function<T, String> jsonEncoder) {
        this.provider = provider;
        this.tsExtractor = tsExtractor;
        this.jsonEncoder = jsonEncoder;
    }

    // ------------------------- Dumper -------------------------
    @Override
    public Iterator<String> dumpAll() {
        List<String> out = new ArrayList<>();
        // Samples
        for (T s : provider.snapshotAll()) {
            String j = jsonEncoder.apply(s);
            if (j != null) out.add(j);
        }
        // Labels
        for (Label l : labels) out.add(l.toJson());
        return out.iterator();
    }

    @Override
    public Iterator<String> dumpSince(long sinceEpochMs) {
        List<String> out = new ArrayList<>();
        for (T s : provider.snapshotAll()) {
            Long t = tsExtractor.apply(s);
            if (t != null && t > sinceEpochMs) {
                String j = jsonEncoder.apply(s);
                if (j != null) out.add(j);
            }
        }
        for (Label l : labels) if (l.t > sinceEpochMs) out.add(l.toJson());
        return out.iterator();
    }

    // ------------------------- Labeler -------------------------
    @Override
    public void addLabel(long epochMs, String text) {
        labels.add(new Label(epochMs, text));
    }

    private static final class Label {
        final long t; final String text;
        Label(long t, String text) { this.t = t; this.text = text; }
        String toJson() {
            // Escape backslashes and double quotes for JSON string safety
            String safe = (text == null) ? ""
                    : text.replace("\\", "\\\\").replace("\"", "\\\"");
            return "{\"t\":" + t + ",\"type\":\"label\",\"text\":\"" + safe + "\"}";
        }

    }
}

