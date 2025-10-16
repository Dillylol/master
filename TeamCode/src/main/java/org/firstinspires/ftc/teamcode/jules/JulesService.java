package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBufferJsonAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;

import java.io.IOException;

/**
 * Manages the lifecycle of the Jules data bridge, ensuring it starts
 * once when the app launches and stops when the app closes.
 * This provides a persistent connection.
 */
public class JulesService {

    private static JulesBuilder julesBuilderInstance = null;
    private static JulesHttpBridge httpBridge = null;
    private static boolean isInitialized = false;

    /**
     * Initializes the Jules service. This should be called once by the JulesHooks class.
     *
     * @param context The application context.
     */
    public static synchronized void init(Context context) {
        if (isInitialized) {
            return;
        }

        // 1. Create the core components for the JULES system.
        JulesBuffer buffer = new JulesBuffer(8192); // A buffer to hold recent data.
        JulesStreamBus streamBus = new JulesStreamBus(); // For live data streaming.
        JulesBufferJsonAdapter adapter = new JulesBufferJsonAdapter(buffer); // Connects the buffer to the HTTP bridge.
        String token = JulesTokenStore.getOrCreate(context); // Get or create a security token.

        try {
            // 2. Start the HTTP Server on port 58080.
            httpBridge = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
        } catch (IOException e) {
            // If the server fails to start, we can't proceed.
            // This error will be visible in the robot controller logs.
            e.printStackTrace();
            return;
        }

        // 3. Create the transmitter that sends data to our buffer.
        // We pass null for TelemetryManager and dsTelemetry because this service runs outside of an OpMode.
        JulesRamTx transmitter = new JulesRamTx(buffer.capacity(), null, null, "jules");

        // 4. Create the single, globally accessible instance of the JulesBuilder.
        julesBuilderInstance = new JulesBuilder(transmitter);

        isInitialized = true;
    }

    /**
     * Provides global access to the single JulesBuilder instance.
     *
     * @return The singleton JulesBuilder object.
     */
    public static JulesBuilder getJules() {
        if (!isInitialized) {
            // This is a safeguard. If initialization failed, this prevents crashes.
            return new JulesBuilder(null);
        }
        return julesBuilderInstance;
    }

    /**
     * Shuts down the Jules service. This is handled automatically.
     */
    public static void stop() {
        if (httpBridge != null) {
            httpBridge.close();
        }
        isInitialized = false;
        julesBuilderInstance = null;
    }
}