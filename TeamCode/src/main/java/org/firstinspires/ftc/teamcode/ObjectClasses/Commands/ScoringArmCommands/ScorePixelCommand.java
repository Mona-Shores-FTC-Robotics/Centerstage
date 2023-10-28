package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;

public final class ScorePixelCommand extends CommandBase {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LiftSlideSubsystem liftSlideSubsystem;
    private final ShoulderSubsystem shoulderSubsystem;

    public ScorePixelCommand() {
        endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();
        shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        addRequirements(endEffectorSubsystem);
        addRequirements(liftSlideSubsystem);
        addRequirements(shoulderSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}