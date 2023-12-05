package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public final class PixelPusherSubsystem extends SubsystemBase {

    public static class PixelPusherParameters {
        public double OUT_OF_WAY_POSITION = .45;
        public double PIXEL_PUSHING_POSITION = .5;
    }

    public static PixelPusherParameters pixelPusherParameters = new PixelPusherParameters();

    public enum PixelPusherStates {
        NOT_PUSHING,
        PUSHING;
        public double position;

        static {
            NOT_PUSHING.position = pixelPusherParameters.OUT_OF_WAY_POSITION;
            PUSHING.position = pixelPusherParameters.PIXEL_PUSHING_POSITION;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public final Servo pixelPusher;
    public PixelPusherStates currentState;

    public PixelPusherSubsystem(final HardwareMap hMap, final String name) {
        pixelPusher = hMap.get(Servo.class, name);
    }

    public void init() {
        SetStatesValues();
        currentState= PixelPusherStates.PUSHING;
    }

    public void initTeleOp() {
        SetStatesValues();
        currentState= PixelPusherStates.NOT_PUSHING;
        pixelPusher.setPosition(PixelPusherStates.NOT_PUSHING.position);
    }

    public void periodic(){
        SetStatesValues();
        MatchConfig.telemetryPacket.put("Pixel Pusher State", currentState);
    }

    private void SetStatesValues() {
        PixelPusherStates.PUSHING.SetState(PixelPusherSubsystem.pixelPusherParameters.PIXEL_PUSHING_POSITION);
        PixelPusherStates.NOT_PUSHING.SetState(PixelPusherSubsystem.pixelPusherParameters.OUT_OF_WAY_POSITION);
    }

}
