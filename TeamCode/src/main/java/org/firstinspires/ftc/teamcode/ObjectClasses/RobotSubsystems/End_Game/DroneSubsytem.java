package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class DroneSubsytem extends SubsystemBase {

    public static class DroneParameters {

        public DroneDeployState DRONE_DEPLOY_STARTING_STATE = DroneDeployState.HOLD;

        public double DRONE_VALUE_THRESHOLD = .03;

        public double HOLD_VALUE = 0;

        public double FLY_VALUE = 1;

    }

    public static  DroneParameters droneParameters = new DroneParameters();



}
