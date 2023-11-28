package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.wifi.NetworkConnection;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;

public class RobotConfig {
    public static String getRobotControllerName() {
        NetworkConnection networkConnection = NetworkConnectionHandler.getInstance().getNetworkConnection();
        if (networkConnection instanceof WifiDirectAssistant) {
            return ((WifiDirectAssistant) networkConnection).getDeviceName();
        }
        return "Unknown";
    }
}