package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public ReleaseDroneCommand(DroneSubsystem subsystem, DroneSubsystem.DroneDeployState inputState) {
        droneSubsystem = subsystem;
        targetState = inputState;
        targetPosition = targetState.position;

        //require this subsystem
        addRequirements(droneSubsystem);
    }

    @Override
    public void initialize() {droneSubsystem.drone.setPosition(targetState.position);}

    public void execute() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current Drone State", droneSubsystem.currentState);
        telemetryPacket.put("Target Drone State", targetState);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public boolean isFinished() {
        boolean done =true;
        if (done)
        {
            droneSubsystem.currentState = targetState;
            return true;
        } else return false;
    }
}
