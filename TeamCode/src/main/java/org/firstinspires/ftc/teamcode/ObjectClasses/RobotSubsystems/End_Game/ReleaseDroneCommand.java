package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;

public class ReleaseDroneCommand extends CommandBase {
    private final DroneSubsystem droneSubsystem;
    private DroneSubsystem.DroneDeployState targetState;

    public ReleaseDroneCommand(DroneSubsystem subsystem, DroneSubsystem.DroneDeployState inputState) {
        droneSubsystem = subsystem;
        targetState = inputState;

        //require this subsystem
        addRequirements(droneSubsystem);
    }

    @Override
    public void initialize() {droneSubsystem.drone.setPosition(targetState.position);}

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        droneSubsystem.currentState = targetState;
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        //change the current state to the target state
        MatchConfig.telemetryPacket.addLine("Drone servo move COMPLETE From " + droneSubsystem.currentState + " to " + targetState);
        droneSubsystem.setCurrentState(targetState);
    }

}
