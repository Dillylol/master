package org.firstinspires.ftc.teamcode.jules.bridge;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import java.util.UUID;

/**
 * Minimal HTTP server w/ no external deps:
 *   - GET  /jules/handshake
 *   - GET  /jules/dump[?since=epochMs]
 *   - POST /jules/label?text=...&t=...
 * Token required for /dump and /label via header X-Jules-Token or query token=.
 */
public class JulesHttpBridgeLite implements AutoCloseable {

    public interface BufferProvider {
        /** Return ALL JSONL lines collected so far. */
        List<String> snapshotAll();
        /** Optional: return only lines with t > sinceEpochMs (if unsupported, return null). */
        List<String> snapshotSince(long sinceEpochMs);
    }

    public interface LabelSink {
        void addLabel(long epochMs, String text);
    }

    private final int port;
    private final BufferProvider provider;
    private final LabelSink labelSink;
    private final String token;

    private volatile boolean running = false;
    private Thread serverThread;

    public JulesHttpBridgeLite(int port, BufferProvider provider, LabelSink labelSink) {
        this.port = port;
        this.provider = provider;
        this.labelSink = labelSink;
        this.token = generateToken();
    }

    public void start() {
        if (running) return;
        running = true;
        serverThread = new Thread(this::runServer, "JulesHttpBridgeLite");
        serverThread.setDaemon(true);
        serverThread.start();
    }

    public String getToken() { return token; }
    public int getPort() { return port; }

    @Override
    public void close() {
        running = false;
        try { if (serverThread != null) serverThread.interrupt(); } catch (Exception ignored) {}
    }

    public String advertiseLine() {
        List<String> ips = localIPv4();
        String ip = ips.isEmpty() ? "0.0.0.0" : ips.get(0);
        return "JULES → http://" + ip + ":" + port + "  token=" + token;
    }

    // ----------------------- Internal -----------------------

    private void runServer() {
        ServerSocket ss = null;
        try {
            ss = new ServerSocket(port);
            ss.setReuseAddress(true);
            while (running) {
                try {
                    Socket s = ss.accept();
                    handleClient(s);
                } catch (IOException ignored) {}
            }
        } catch (IOException e) {
            // swallow; caller can surface via telemetry
        } finally {
            if (ss != null) try { ss.close(); } catch (IOException ignored) {}
        }
    }

    private void handleClient(Socket socket) {
        try (Socket s = socket) {
            InputStream in = s.getInputStream();
            OutputStream out = s.getOutputStream();
            BufferedReader r = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
            BufferedWriter w = new BufferedWriter(new OutputStreamWriter(out, StandardCharsets.UTF_8));

            // Request line
            String req = r.readLine();
            if (req == null || req.length() == 0) return; // drop
            // Example: "GET /jules/handshake?x=y HTTP/1.1"
            String[] parts = req.split(" ");
            if (parts.length < 2) { writeJson(w, 400, "{\"ok\":false,\"error\":\"bad request\"}"); return; }
            String method = parts[0];
            String pathAndQuery = parts[1];

            // Headers
            Map<String, String> headers = new HashMap<>();
            String line;
            int contentLength = 0;
            while ((line = r.readLine()) != null && line.length() > 0) {
                int idx = line.indexOf(':');
                if (idx > 0) {
                    String k = line.substring(0, idx).trim().toLowerCase(Locale.US);
                    String v = line.substring(idx + 1).trim();
                    headers.put(k, v);
                    if (k.equals("content-length")) {
                        try { contentLength = Integer.parseInt(v); } catch (Exception ignored) {}
                    }
                }
            }

            // Body (only if POST and Content-Length > 0)
            String body = null;
            if ("POST".equals(method) && contentLength > 0) {
                char[] buf = new char[contentLength];
                int read = r.read(buf);
                body = new String(buf, 0, Math.max(0, read));
            }

            // Split path & query
            String path = pathAndQuery;
            String query = "";
            int q = pathAndQuery.indexOf('?');
            if (q >= 0) {
                path = pathAndQuery.substring(0, q);
                query = pathAndQuery.substring(q + 1);
            }
            Map<String, String> params = parseQuery(query);
            if (body != null && body.contains("=")) {
                // also parse body as form-encoded
                Map<String, String> bodyParams = parseQuery(body);
                params.putAll(bodyParams);
            }

            boolean handshake = "GET".equals(method) && "/jules/handshake".equals(path);
            if (!handshake) {
                String qTok = params.get("token");
                String hTok = headers.get("x-jules-token");
                if (!token.equals(qTok) && !token.equals(hTok)) {
                    writeJson(w, 401, "{\"ok\":false,\"error\":\"unauthorized\"}");
                    return;
                }
            }

            if ("GET".equals(method) && "/jules/handshake".equals(path)) {
                writeJson(w, 200, "{\"ok\":true,\"token\":\"" + token + "\",\"time\":\"" + nowIso() + "\"}");
                return;
            }

            if ("GET".equals(method) && "/jules/dump".equals(path)) {
                long since = parseLongOr(params.get("since"), -1);
                List<String> lines;
                if (since > 0 && provider.snapshotSince(since) != null) lines = provider.snapshotSince(since);
                else lines = safe(provider.snapshotAll());
                writeJsonl(w, 200, lines);
                return;
            }

            if ("/jules/label".equals(path)) {
                if (!"GET".equals(method) && !"POST".equals(method)) {
                    writeJson(w, 405, "{\"ok\":false,\"error\":\"method\"}");
                    return;
                }
                String text = firstNonEmpty(params.get("text"), params.get("label"));
                if (text == null || text.trim().isEmpty()) {
                    writeJson(w, 400, "{\"ok\":false,\"error\":\"missing text\"}");
                    return;
                }
                long t = parseLongOr(params.get("t"), System.currentTimeMillis());
                if (labelSink != null) labelSink.addLabel(t, text);
                writeJson(w, 200, "{\"ok\":true}");
                return;
            }

            writeJson(w, 404, "{\"ok\":false,\"error\":\"not found\"}");
        } catch (IOException ignored) {
        }
    }

    private static List<String> safe(List<String> in) {
        return in == null ? Collections.<String>emptyList() : in;
    }

    private static Map<String, String> parseQuery(String q) {
        Map<String, String> map = new HashMap<>();
        if (q == null || q.isEmpty()) return map;
        String[] pairs = q.split("&");
        for (String p : pairs) {
            int i = p.indexOf('=');
            if (i < 0) continue;
            String k = urlDecode(p.substring(0, i));
            String v = urlDecode(p.substring(i + 1));
            map.put(k, v);
        }
        return map;
    }

    private static String urlDecode(String s) {
        try { return URLDecoder.decode(s, "UTF-8"); } catch (Exception e) { return s; }
    }

    private static void writeJson(BufferedWriter w, int code, String json) throws IOException {
        byte[] body = json.getBytes(StandardCharsets.UTF_8);
        writeHead(w, code, "application/json; charset=utf-8", body.length);
        w.write(json);
        w.flush();
    }

    private static void writeJsonl(BufferedWriter w, int code, List<String> lines) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (String ln : lines) {
            if (ln == null) continue;
            if (!ln.endsWith("\n")) sb.append(ln).append('\n'); else sb.append(ln);
        }
        byte[] bytes = sb.toString().getBytes(StandardCharsets.UTF_8);
        writeHead(w, code, "application/jsonl; charset=utf-8", bytes.length);
        w.write(sb.toString());
        w.flush();
    }

    private static void writeHead(BufferedWriter w, int code, String ctype, int length) throws IOException {
        w.write("HTTP/1.1 " + code + "\r\n");
        w.write("Content-Type: " + ctype + "\r\n");
        w.write("Content-Length: " + length + "\r\n");
        w.write("Connection: close\r\n");
        w.write("\r\n");
    }

    private static String nowIso() {
        SimpleDateFormat fmt = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US);
        fmt.setTimeZone(TimeZone.getTimeZone("UTC"));
        return fmt.format(new Date());
    }

    private static String generateToken() {
        return Long.toString(UUID.randomUUID().getMostSignificantBits() & Long.MAX_VALUE, 36)
                + Long.toString(UUID.randomUUID().getLeastSignificantBits() & Long.MAX_VALUE, 36).substring(0, 4);
    }

    public static List<String> localIPv4() {
        List<String> ips = new ArrayList<>();
        try {
            Enumeration<NetworkInterface> nets = NetworkInterface.getNetworkInterfaces();
            while (nets.hasMoreElements()) {
                NetworkInterface ni = nets.nextElement();
                if (!ni.isUp() || ni.isLoopback()) continue;
                Enumeration<InetAddress> addrs = ni.getInetAddresses();
                while (addrs.hasMoreElements()) {
                    InetAddress a = addrs.nextElement();
                    if (a instanceof Inet4Address && a.isSiteLocalAddress()) ips.add(a.getHostAddress());
                }
            }
        } catch (SocketException ignored) {}
        return ips;
    }

    // ---- tiny helpers (avoid external deps/static imports) ----
    private static long parseLongOr(String s, long def) {
        try { return (s == null) ? def : Long.parseLong(s); } catch (Exception e) { return def; }
    }
    private static String firstNonEmpty(String a, String b) {
        if (a != null && !a.trim().isEmpty()) return a;
        return (b != null && !b.trim().isEmpty()) ? b : null;
    }
}
