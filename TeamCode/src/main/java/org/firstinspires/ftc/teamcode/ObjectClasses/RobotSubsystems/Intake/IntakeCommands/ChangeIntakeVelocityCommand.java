package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public ChangeIntakeVelocityCommand(IntakeSubsystem subsystem, IntakeSubsystem.IntakeStates inputState) {
        intakeSubsystem = subsystem;
        targetState = inputState;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        intakeSubsystem.intake.setVelocity(targetState.velocity);
    }

    public void execute() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        currentVelocity = intakeSubsystem.intake.getVelocity();
        telemetryPacket.put("Target Intake State", targetState);
        telemetryPacket.put("Target Intake Velocity", targetState.velocity);
        telemetryPacket.put("Current Intake State", intakeSubsystem.currentState);
        telemetryPacket.put("Current Intake Velocity", currentVelocity);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    //this could perhaps end only after reaching the target velocity, but we would need to figure that out
    @Override
    public boolean isFinished() {
        return true;
    }
}