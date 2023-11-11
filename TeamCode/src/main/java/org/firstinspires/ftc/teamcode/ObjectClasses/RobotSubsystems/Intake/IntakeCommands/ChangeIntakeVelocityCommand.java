package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ChangeIntakeVelocityCommand extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem intakeSubsystem;

    //declare target state
    private IntakeSubsystem.IntakeStates targetState;
    private double currentVelocity;

    Telemetry telemetry;

    public ChangeIntakeVelocityCommand(IntakeSubsystem subsystem, IntakeSubsystem.IntakeStates inputState) {
        intakeSubsystem = subsystem;
        targetState = inputState;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        intakeSubsystem.intake.setVelocity(targetState.velocity);
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.clearAll();
    }

    public void execute() {
        currentVelocity = intakeSubsystem.intake.getVelocity();
        telemetry.addData("Target Intake State", targetState);
        telemetry.addData("Target Intake Velocity", targetState.velocity);
        telemetry.addData("Current Intake State", intakeSubsystem.currentState);
        telemetry.addData("Current Intake Velocity", currentVelocity);
    }

}