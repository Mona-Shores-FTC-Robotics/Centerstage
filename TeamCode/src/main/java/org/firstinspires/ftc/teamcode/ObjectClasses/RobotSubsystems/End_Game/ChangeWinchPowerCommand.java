package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ChangeWinchPowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private ClimberSubsystem.WinchMotorStates targetState;
    private double currentVelocity;

    public ChangeWinchPowerCommand(ClimberSubsystem subsystem, ClimberSubsystem.WinchMotorStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
    }
    @Override
    public void initialize() {
        climberSubsystem.winchMotor.setPower(targetState.power);
    }

    public void execute() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        currentVelocity = climberSubsystem.winchMotor.getVelocity();
        telemetryPacket.put("Target Winch State", targetState);
        telemetryPacket.put("Current Winch State", climberSubsystem.currentState);
        telemetryPacket.put("Current Winch Velocity", currentVelocity);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    //this only needs to run once to change teh state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }


}