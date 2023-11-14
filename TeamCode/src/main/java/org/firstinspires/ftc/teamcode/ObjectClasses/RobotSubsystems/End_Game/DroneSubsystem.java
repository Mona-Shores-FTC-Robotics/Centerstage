package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class DroneSubsystem extends SubsystemBase {

    public static class DroneParameters {

        public DroneDeployState DRONE_DEPLOY_STARTING_STATE = DroneDeployState.HOLD;

        public double DRONE_VALUE_THRESHOLD = .03;

        public double HOLD_VALUE = .5;

        public double FLY_VALUE = 1;
        public double END_GAME_TIME = 120;

    }

    public static  DroneParameters droneParameters = new DroneParameters();

    public enum DroneDeployState {
        HOLD (.5),
        FLY (1);
        public double position;
        DroneDeployState(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public Servo drone;
    public DroneSubsystem.DroneDeployState currentState;
    public double currentPosition;
    private GamepadHandling gamepadHandling;

    public DroneSubsystem(final HardwareMap hMap, final String name) {
        drone = hMap.servo.get("drone");

    }

    public void init() {
        DroneDeployState.HOLD.SetState(droneParameters.HOLD_VALUE);
        DroneDeployState.FLY.SetState(droneParameters.FLY_VALUE);

        currentState = droneParameters.DRONE_DEPLOY_STARTING_STATE;
        currentPosition = currentState.position;
    }

    public void periodic(){

        //
//        if (MatchConfig.teleOpTimer.seconds() > droneParameters.END_GAME_TIME)
//        {
//            gamepadHandling.getOperatorGamepad().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                    .whenPressed(new ReleaseDroneCommand(this, DroneDeployState.FLY));
//        }

        DroneDeployState.HOLD.SetState(droneParameters.HOLD_VALUE);
        DroneDeployState.FLY.SetState(droneParameters.FLY_VALUE);

    }


}
