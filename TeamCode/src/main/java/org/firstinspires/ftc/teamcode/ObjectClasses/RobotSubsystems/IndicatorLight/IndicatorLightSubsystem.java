package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IndicatorLight;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndicatorLightSubsystem extends SubsystemBase {

    public static DigitalChannel red;
    public static DigitalChannel green;
    private DigitalChannel touch;
    public LightStates currentState;

    public enum LightStates {
        OFF,
        RED,
        GREEN,
        AMBER,
    }


    public IndicatorLightSubsystem(final HardwareMap hMap, String name) {
        red = hMap.get(DigitalChannel.class, "red");
        green = hMap.get(DigitalChannel.class, "green");
        touch = hMap.get(DigitalChannel.class, "touch");
    }

    public void init() {
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        currentState = LightStates.OFF;
    }

    public void periodic() {
        if (touch.getState()){
         new IndicatorLightChangeCommand(this);
        }
    }
}
