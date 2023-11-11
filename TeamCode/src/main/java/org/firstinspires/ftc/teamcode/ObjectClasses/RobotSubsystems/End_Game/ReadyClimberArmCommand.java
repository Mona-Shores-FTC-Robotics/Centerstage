package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ReadyClimberArmCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state & position
    private ClimberSubsystem.ClimberArmStates targetState;
    private double targetPosition;

    public ReadyClimberArmCommand(ClimberSubsystem subsystem, ClimberSubsystem.ClimberArmStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
        targetPosition = targetState.position;

        //require this subsystem
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.climberArm.setPosition(targetState.position);
        //create a new telemetry packet for this command

    }

    public void execute() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        climberSubsystem.currentClimberArmPosition = climberSubsystem.climberArm.getPosition();
        telemetryPacket.put("Current ClimberArm State", climberSubsystem.currentClimberArmState);
        telemetryPacket.put("Current Position", climberSubsystem.currentClimberArmPosition);
        telemetryPacket.put("Target ClimberArm State", targetState);
        telemetryPacket.put("Target Position", targetPosition);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public boolean isFinished() {
        boolean done = Math.abs( climberSubsystem.currentClimberArmPosition-targetPosition) < ClimberSubsystem.climberParameters.CLIMBER_ARM_THRESHOLD;
        if (done)
        {
            climberSubsystem.currentClimberArmState = targetState;
            return true;
        } else return false;
    }
}
