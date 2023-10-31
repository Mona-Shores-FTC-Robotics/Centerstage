package org.firstinspires.ftc.teamcode.ObjectClasses.Commands;


import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Actions.CenterstageActions;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.ActionAsCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DriveWithConstantHeading;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.ActuateEndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.lang.reflect.Field;

public abstract class CenterstageCommands {

    public static Command defaultCommand;
    public static Command turnTo0;
    public static Command turnTo90;
    public static Command turnTo180;
    public static Command turnTo270;

    public static Command driveLockedTo90;
    public static Command rotateBackDrop;
    public static Command rotateBackdrop;
    public static Command rotateIntake;
    public static Command openClaw;
    public static Command closeClaw;
    public static Command liftHome;
    public static Command liftLow;
    public static Command liftMid;
    public static Command liftHigh;
    public static Command wait;

    public static SequentialCommandGroup scorePixel;

    private static DriveSubsystem driveSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static EndEffectorSubsystem endEffectorSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static MecanumDriveMona mecanumDrive;

    public static void MakeTeleOpCommands() {
//        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
//        endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();
//        mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        //this command broke things when the subsystem was not Robot.getInstance().getDriveSubsystem - and instead just used driveSubsystem from above.
//        defaultCommand = new DefaultDrive(driveSubsystem,
//                GamepadHandling.getDriverGamepad()::getLeftY,
//                GamepadHandling.getDriverGamepad()::getLeftX,
//                GamepadHandling.getDriverGamepad()::getRightX
//        );

//        turnTo0 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
//                GamepadHandling.getDriverGamepad()::getLeftY,
//                GamepadHandling.getDriverGamepad()::getLeftX,
//                0));
//
//        turnTo90 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
//                GamepadHandling.getDriverGamepad()::getLeftY,
//                GamepadHandling.getDriverGamepad()::getLeftX,
//                90));
//
//        turnTo180 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
//                GamepadHandling.getDriverGamepad()::getLeftY,
//                GamepadHandling.getDriverGamepad()::getLeftX,
//                180));
//
//        turnTo270 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
//                GamepadHandling.getDriverGamepad()::getLeftY,
//                GamepadHandling.getDriverGamepad()::getLeftX,
//                270));

        rotateBackdrop = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        rotateIntake = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

//        openClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.OPEN);
//
//        closeClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.CLOSED);

        liftHome = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        liftLow = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        liftMid = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        liftHigh = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HIGH);


        wait = new WaitCommand(2000);

    }

    public static Command MakeScorePixelCommand() {
        scorePixel = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        liftMid,
                        rotateBackdrop),
                wait,
                openClaw,
                wait,
                closeClaw,
                wait,
                rotateIntake,
                wait,
                liftHome
        );
        return scorePixel;
    }




    public static ParallelRaceGroup BackupFromBlueBackdropCommand() {
        ParallelRaceGroup seqCommand = new ParallelRaceGroup(
                new ActionAsCommand(Robot.getInstance().getDriveSubsystem(), CenterstageActions.backUpFromBlueBackdrop()),
                new IsGamepadActiveCommand()
        );
        return seqCommand;
    }

    public static ParallelRaceGroup BackupFromRedBackdropCommand() {
        ParallelRaceGroup seqCommand = new ParallelRaceGroup(
                new ActionAsCommand(Robot.getInstance().getDriveSubsystem(), CenterstageActions.backUpFromRedBackdrop()),
                new IsGamepadActiveCommand()
        );
        return seqCommand;
    }
}
