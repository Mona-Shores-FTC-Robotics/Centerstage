package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class ChangeIntakeStateVelocity extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem intakeSubsystem;

    //declare target state
    private IntakeSubsystem.IntakeStates targetState;
    private DoubleSupplier targetVelocity;
    private double currentVelocity;

    TelemetryPacket telemetryPacket;

    public ChangeIntakeStateVelocity(IntakeSubsystem subsystem, IntakeSubsystem.IntakeStates inputState, DoubleSupplier vel) {
        intakeSubsystem = subsystem;
        targetState = inputState;
        targetVelocity = vel;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        if (targetState== IntakeSubsystem.IntakeStates.INTAKE_REVERSE){
            intakeSubsystem.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        } else intakeSubsystem.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeSubsystem.intake.setVelocity(targetVelocity.getAsDouble());

        //create a new telemetry packet for this command
        telemetryPacket = new TelemetryPacket();
    }

    public void execute() {
        intakeSubsystem.intake.setVelocity(targetVelocity.getAsDouble());
        currentVelocity = intakeSubsystem.intake.getVelocity();

        telemetryPacket.put("Target Intake State", targetState);
        telemetryPacket.put("Target Intake Velocity", targetVelocity.getAsDouble());
        telemetryPacket.put("Current Intake State", intakeSubsystem.currentState);
        telemetryPacket.put("Current Intake Velocity", currentVelocity);
    }

}