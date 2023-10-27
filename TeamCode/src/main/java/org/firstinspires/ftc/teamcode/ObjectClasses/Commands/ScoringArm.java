package org.firstinspires.ftc.teamcode.ObjectClasses.Commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;

public final class ScoringArm {
    EndEffectorSubsystem endEffectorSubsystem;
    LiftSlideSubsystem liftSlideSubsystem;
    ShoulderSubsystem shoulderSubsystem;
    public ScoringArm() {

    }

    public void init() {
        endEffectorSubsystem = Robot.getInstance().getScoringArm().endEffectorSubsystem;
        liftSlideSubsystem.init();
        shoulderSubsystem.init();
    }

    public Action makeGrabAndScorePixelOnBackdropMid() {
        Action test1 = new SequentialAction(
                endEffectorSubsystem.actuate(EndEffectorSubsystem.EndEffectorStates.OPEN),
                new SleepAction(1),
                endEffectorSubsystem.actuate(EndEffectorSubsystem.EndEffectorStates.CLOSED),
                new SleepAction(1),
                endEffectorSubsystem.actuate(EndEffectorSubsystem.EndEffectorStates.OPEN),
                new SleepAction(1),
                endEffectorSubsystem.actuate(EndEffectorSubsystem.EndEffectorStates.CLOSED));
        return test1;
    }


    public Action grabAndScorePixelOnBackdropLow() {
        Action test2 = new SequentialAction(
                liftSlideSubsystem.liftToMidHeight(),
                liftSlideSubsystem.liftToHighHeight(),
                liftSlideSubsystem.liftToLowHeight(),
                liftSlideSubsystem.liftToHighHeight()
        );
        return test2;
    }
}