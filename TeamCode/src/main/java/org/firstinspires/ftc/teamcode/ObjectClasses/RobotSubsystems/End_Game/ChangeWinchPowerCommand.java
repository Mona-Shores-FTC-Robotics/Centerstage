package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ChangeWinchPowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private ClimberSubsystem.WinchMotorStates targetState;

    public ChangeWinchPowerCommand(ClimberSubsystem subsystem, ClimberSubsystem.WinchMotorStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
    }

    @Override
    public void initialize() {

        climberSubsystem.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberSubsystem.winchMotor.setPower(targetState.power);
    }

    public void execute() {
    }

    //this only needs to run once to change the state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("End Game Winch changed from" + climberSubsystem.currentWinchMotorState + " to " + targetState);
        //change the current state to the target state
        climberSubsystem.setCurrentWinchMotorState(targetState);
    }
}