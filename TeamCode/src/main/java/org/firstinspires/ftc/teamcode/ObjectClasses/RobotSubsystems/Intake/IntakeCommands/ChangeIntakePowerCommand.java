package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.LIFT_HEIGHT_TICK_THRESHOLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ChangeIntakePowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem intakeSubsystem;

    //declare target state
    private IntakeSubsystem.IntakeStates targetState;
    private IntakeSubsystem.IntakeStates targetState2;
    private double currentVelocity;

    public ChangeIntakePowerCommand(IntakeSubsystem subsystem, IntakeSubsystem.IntakeStates inputState, IntakeSubsystem.IntakeStates inputState2) {
        intakeSubsystem = subsystem;
        targetState = inputState;
        targetState2 = inputState2;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        intakeSubsystem.intake.setPower(targetState.power);
        intakeSubsystem.intake2.setPower(targetState2.power);
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

    //this only needs to run once to change teh state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }


}