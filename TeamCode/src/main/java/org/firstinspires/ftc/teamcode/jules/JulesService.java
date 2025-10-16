package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Manages the lifecycle of the Jules data bridge, ensuring it starts
 * once when the app launches and stops when the app closes.
 * This provides a persistent connection, similar to FTCLib Panels.
 */
public class JulesService {

    private static JulesBuilder julesBuilderInstance = null;
    private static boolean isInitialized = false;

    /**
     * Initializes the Jules service. This should be called once from the
     * FtcRobotControllerActivity's onCreate method.
     *
     * @param context The application context.
     */
    public static synchronized void init(Context context) {
        if (isInitialized) {
            return;
        }

        // The JulesBridgeManager needs access to the WebHandlerManager to start its server.
        // We can get it from the AppUtil context.
        WebHandlerManager webHandlerManager = WebHandlerManager.getInstance();

        // Initialize the bridge manager, which starts the web server.
        JulesRamTx transmitter = JulesBridgeManager.init(webHandlerManager);

        // Create a single, globally accessible instance of the JulesBuilder.
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
            // This is a safeguard. The service should be initialized by the Activity.
            // Returning a "dummy" builder prevents crashes if initialization fails,
            // though no data will be sent.
            return new JulesBuilder(null);
        }
        return julesBuilderInstance;
    }

    /**
     * Shuts down the Jules service. This is typically handled automatically
     * by the Android application lifecycle.
     */
    public static void stop() {
        JulesBridgeManager.stop();
        isInitialized = false;
        julesBuilderInstance = null;
    }
}
