package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderCommand extends CommandBase {

    // The subsystem the command runs on
    private final ShoulderSubsystem shoulderSubsystem;

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;
    TelemetryPacket telemetryPacket;

    public RotateShoulderCommand(ShoulderSubsystem subsystem, ShoulderSubsystem.ShoulderStates inputState) {
            shoulderSubsystem = subsystem;
            targetState = inputState;
            targetPosition = targetState.position;

            //require this subsystem
            addRequirements(shoulderSubsystem);
        }

    @Override
    public void initialize() {
        shoulderSubsystem.shoulder.setPosition(targetState.position);
        //create a new telemetry packet for this command
        telemetryPacket = new TelemetryPacket();
    }

    public void execute() {
        shoulderSubsystem.currentPosition = shoulderSubsystem.shoulder.getPosition();
        telemetryPacket.put("Current Shoulder State", shoulderSubsystem.currentState);
        telemetryPacket.put("Current Position", shoulderSubsystem.currentPosition);
        telemetryPacket.put("Target Shoulder State", targetState);
        telemetryPacket.put("Target Position", targetPosition);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public boolean isFinished() {
        boolean done = Math.abs( shoulderSubsystem.currentPosition-targetPosition) < ShoulderSubsystem.shoulderParameters.SHOULDER_VALUE_THRESHOLD;
            if (done)
            {
                shoulderSubsystem.currentState = targetState;
                return true;
            } else return false;
    }
}
