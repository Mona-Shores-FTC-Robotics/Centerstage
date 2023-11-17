package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ReadyClimberArmCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state & position
    private final ClimberSubsystem.ClimberArmStates targetState;


    public ReadyClimberArmCommand(ClimberSubsystem subsystem, ClimberSubsystem.ClimberArmStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;

        //require this subsystem
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.climberArm.setPosition(targetState.position);
    }

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("Climber Arm Move COMPLETE From " + climberSubsystem.currentClimberArmState + " to " + targetState);
        //change the current state to the target state
        climberSubsystem.setCurrentClimberArmState(targetState);
    }
}
