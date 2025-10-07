// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesHttpBridge.java
package org.firstinspires.ftc.teamcode.jules.bridge;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

public class JulesHttpBridge extends NanoHTTPD implements AutoCloseable {
    private final String token;
    private final Dumper dumper;
    private final Labeler labeler;
    private final JulesStreamBus streamBus;
    private String ip = "N/A";

    // --- RESTORED INTERFACES ---
    // These define the contract for how the bridge dumps data and adds labels.
    public interface Dumper {
        Iterator<String> dumpAll();
        Iterator<String> dumpSince(long sinceMs);
    }
    public interface Labeler {
        void addLabel(long tMillis, String text);
    }
    // ---------------------------

    /**
     * The complete constructor. It now correctly uses the Dumper and Labeler interfaces.
     */
    public JulesHttpBridge(int port, Dumper dumper, Labeler labeler, String token, JulesStreamBus streamBus) throws IOException {
        super(port);
        this.token = token;
        this.dumper = dumper;
        this.labeler = labeler;
        this.streamBus = streamBus;
        findIP();
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        System.out.println("JULES: HTTP bridge running on http://" + ip + ":" + port);
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();
        Method method = session.getMethod();
        Map<String, String> params = session.getParms();

        try {
            // Universal token check for all endpoints
            if (params.get("token") == null || !params.get("token").equals(token)) {
                return newFixedLengthResponse(Response.Status.UNAUTHORIZED, "application/json", "{\"ok\":false,\"error\":\"invalid token\"}");
            }

            // --- ENDPOINTS ---

            if ("/jules/label".equals(uri) && params.containsKey("label")) {
                // This now correctly uses the Labeler interface
                labeler.addLabel(System.currentTimeMillis(), params.get("label"));
                return newFixedLengthResponse(Response.Status.OK, "application/json", "{\"ok\":true}");
            }

            if ("/jules/dump".equals(uri)) {
                // This now correctly uses the Dumper interface
                final Iterator<String> iter;
                if (params.containsKey("since")) {
                    iter = dumper.dumpSince(Long.parseLong(params.get("since")));
                } else {
                    iter = dumper.dumpAll();
                }

                // Create a response from the iterator, formatting as JSON Lines
                InputStream is = new InputStream() {
                    private byte[] currentLine = null;
                    private int index = 0;

                    @Override
                    public int read() throws IOException {
                        if (currentLine == null || index >= currentLine.length) {
                            if (!iter.hasNext()) {
                                return -1; // End of stream
                            }
                            currentLine = (iter.next() + "\n").getBytes("UTF-8");
                            index = 0;
                        }
                        return currentLine[index++];
                    }
                };
                return newChunkedResponse(Response.Status.OK, "application/jsonl; charset=utf-8", is);
            }

            if ("/jules/update-constants".equals(uri) && method == Method.POST) {
                final HashMap<String, String> files = new HashMap<>();
                session.parseBody(files);
                final String jsonBody = files.get("postData");
                Gson gson = new Gson();
                Map<String, Double> updates = gson.fromJson(jsonBody, new TypeToken<Map<String, Double>>(){}.getType());
                for (Map.Entry<String, Double> entry : updates.entrySet()) {
                    try {
                        Field field = Constants.class.getField(entry.getKey());
                        if (java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                            field.set(null, entry.getValue());
                        }
                    } catch (Exception e) {
                        System.out.println("JULES: Could not update constant '" + entry.getKey() + "': " + e.getMessage());
                    }
                }
                return newFixedLengthResponse(Response.Status.OK, "application/json", "{\"ok\":true}");
            }

            if ("/jules/stream".equals(uri) && method == Method.GET) {
                InputStream dataStream = new InputStream() {
                    private final JulesStreamBus.Subscription sub = streamBus.subscribe();
                    private byte[] currentData = null;
                    private int dataIndex = 0;
                    @Override
                    public int read() throws IOException {
                        if (currentData == null || dataIndex >= currentData.length) {
                            try {
                                String line = sub.take();
                                if (line == null) return -1;
                                currentData = ("data: " + line + "\n\n").getBytes("UTF-8");
                                dataIndex = 0;
                            } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                                return -1;
                            }
                        }
                        return currentData[dataIndex++] & 0xFF;
                    }
                    @Override public void close() throws IOException { sub.close(); super.close(); }
                };
                Response r = newChunkedResponse(Response.Status.OK, "text/event-stream; charset=utf-8", dataStream);
                r.addHeader("Cache-Control", "no-cache");
                r.addHeader("Connection", "keep-alive");
                r.addHeader("Access-Control-Allow-Origin", "*");
                return r;
            }

            return newFixedLengthResponse(Response.Status.NOT_FOUND, "application/json", "{\"ok\":false,\"error\":\"not found\"}");

        } catch (Exception e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "application/json", "{\"ok\":false, \"error\":\"" + e.getClass().getSimpleName() + "\"}");
        }
    }

    public String advertiseLine() {
        return "JULES → http://" + ip + ":" + getListeningPort() + "  token=" + token;
    }

    private void findIP() {
        try {
            for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                if (iface.isLoopback() || !iface.isUp()) continue;
                for (InetAddress addr : Collections.list(iface.getInetAddresses())) {
                    if (addr instanceof Inet4Address) {
                        String name = iface.getDisplayName().toLowerCase();
                        if (name.contains("wlan") || name.contains("wireless") || name.contains("wi-fi")) {
                            ip = addr.getHostAddress();
                            return;
                        }
                    }
                }
            }
        } catch (SocketException e) {
            ip = "Error";
        }
    }

    @Override
    public void close() {
        stop();
    }
}