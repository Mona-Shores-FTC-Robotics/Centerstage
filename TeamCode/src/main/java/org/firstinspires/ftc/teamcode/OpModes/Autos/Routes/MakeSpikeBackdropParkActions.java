package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

public class MakeSpikeBackdropParkActions {

    MecanumDriveMona drive;

    public MakeSpikeBackdropParkActions() {
        drive= Robot.getInstance().getDriveSubsystem().mecanumDrive;
        }


    public Action MakeReadyToScorePixelAction() {
        return new SequentialAction(
                        new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED),
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE),
                        new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP),
                        new SleepAction(.5),
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.MID)
                );
    }




    public Action MakeRetractArmAction() {
        return new SequentialAction(
                new ParallelAction(
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE),
                        new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED),
                        new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY)
                ),
                new SleepAction(.5),
                new ParallelAction(
                        new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE),
                        new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.HOME)
                )
        );
    }
}
