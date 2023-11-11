package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;

public class ReleaseDroneCommand extends CommandBase {

    // The subsystem the command runs on
    private final DroneSubsystem droneSubsystem;

    //declare target state & position
    private DroneSubsystem.DroneDeployState targetState;
    private double targetPosition;

    Telemetry telemetry;

    public ReleaseDroneCommand(DroneSubsystem subsystem, DroneSubsystem.DroneDeployState inputState) {
        droneSubsystem = subsystem;
        targetState = inputState;
        targetPosition = targetState.position;

        //require this subsystem
        addRequirements(droneSubsystem);
    }

    @Override
    public void initialize() {
        droneSubsystem.drone.setPosition(targetState.position);
        //create a new telemetry packet for this command
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
    }

    public void execute() {
        droneSubsystem.currentPosition = droneSubsystem.drone.getPosition();
        telemetry.addData("Current Drone State", droneSubsystem.currentState);
        telemetry.addData("Current Position", droneSubsystem.currentPosition);
        telemetry.addData("Target Drone State", targetState);
        telemetry.addData("Target Position", targetPosition);
    }

    @Override
    public boolean isFinished() {
        boolean done = Math.abs( droneSubsystem.currentPosition-targetPosition) < DroneSubsystem.droneParameters.DRONE_VALUE_THRESHOLD;
        if (done)
        {
            droneSubsystem.currentState = targetState;
            return true;
        } else return false;
    }
}
