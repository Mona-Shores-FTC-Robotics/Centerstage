package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class DroneSubsystem extends SubsystemBase {

    public static class DroneParameters {
        public DroneDeployState DRONE_DEPLOY_STARTING_STATE = DroneDeployState.HOLD;
        public double HOLD_VALUE = .35;
        public double FLY_VALUE = 1;
    }

    public static  DroneParameters droneParameters = new DroneParameters();

    public enum DroneDeployState {
        HOLD (.35),
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

    public void setCurrentState(DroneDeployState state) {
        currentState = state;
    }

    public DroneSubsystem(final HardwareMap hMap, final String name) {
        drone = hMap.servo.get("drone");
    }

    public void init() {
        currentState = droneParameters.DRONE_DEPLOY_STARTING_STATE;
    }

    public void periodic(){
        DroneDeployState.HOLD.SetState(droneParameters.HOLD_VALUE);
        DroneDeployState.FLY.SetState(droneParameters.FLY_VALUE);

        //Add the Drone State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Drone State", currentState);
    }


}
