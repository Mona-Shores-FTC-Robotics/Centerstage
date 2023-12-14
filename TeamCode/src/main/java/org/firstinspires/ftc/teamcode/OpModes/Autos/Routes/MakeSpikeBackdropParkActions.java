package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

public class MakeSpikeBackdropParkActions {

    MecanumDriveMona drive;

    public MakeSpikeBackdropParkActions() {
        drive= Robot.getInstance().getDriveSubsystem().mecanumDrive;
        }


    public Action MakeReadyToScorePixelAction(LiftSlideSubsystem.LiftStates liftHeight) {
        return new SequentialAction(
                        new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE),
                        new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP),
                        new SleepAction(.5),
                        new MoveLiftSlideActionFinishImmediate(liftHeight)
                );
    }


    public Action MakeRetractArmAction() {
        return new SequentialAction(
                new ParallelAction(
                        new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY),
                        new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE)
                ),
                new SleepAction(.25),
                new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE_VALUE_STAGING),
                new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.HOME),
                new SleepAction(.25),
                new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE)
        );
    }


}
