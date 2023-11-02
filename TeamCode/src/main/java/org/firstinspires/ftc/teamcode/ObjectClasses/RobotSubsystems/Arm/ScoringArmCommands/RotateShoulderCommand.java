package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderCommand extends CommandBase {

    // The subsystem the command runs on
    private final ShoulderSubsystem shoulderSubsystem;

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;

    Telemetry telemetry;

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
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
    }

    public void execute() {
        shoulderSubsystem.currentPosition = shoulderSubsystem.shoulder.getPosition();

        telemetry.addData("Current Shoulder State", shoulderSubsystem.currentState);
        telemetry.addData("Current Position", shoulderSubsystem.currentPosition);
        telemetry.addData("Target Shoulder State", targetState);
        telemetry.addData("Target Position", targetPosition);

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

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            //what should we do if interrupted?
        }
    }
}
