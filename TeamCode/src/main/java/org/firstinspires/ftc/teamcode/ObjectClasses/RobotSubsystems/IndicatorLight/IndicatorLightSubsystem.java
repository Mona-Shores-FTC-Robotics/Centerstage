package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IndicatorLight;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndicatorLightSubsystem extends SubsystemBase {

    public static DigitalChannel red;
    public static DigitalChannel green;
    private DigitalChannel touch;

    public enum lightStates {
        OFF,
        RED,
        GREEN,
        AMBER,

    }


    public IndicatorLightSubsystem(final HardwareMap hMap, final String name) {
        red = hMap.get(DigitalChannel.class, name);
        green = hMap.get(DigitalChannel.class, name);
        touch = hMap.get(DigitalChannel.class, name);

    }


    public void init() {
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);

    }

    public void IndicatorLightStates() {

        switch (lightStatesNew) {
            case OFF:
                red.setState(true);
                green.setState(true);
                break;
            case RED:
                red.setState(true);
                green.setState(false);
                break;
            case GREEN:
                red.setState(false);
                green.setState(true);
                break;
            case AMBER:
                red.setState(false);
                green.setState(false);
                break;

        }


    }
}
