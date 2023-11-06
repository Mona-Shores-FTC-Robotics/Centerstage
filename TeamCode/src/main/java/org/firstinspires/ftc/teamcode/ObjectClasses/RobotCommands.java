package org.firstinspires.ftc.teamcode.ObjectClasses;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.DefaultLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromBlueBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromRedBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IndicatorLight.IndicatorLightChangeCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IndicatorLight.IndicatorLightSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public abstract class RobotCommands {
    public static Command defaultDriveCommand;
    public static Command defaultLiftSlideCommand;
    public static Command driveWhileAt0Heading;
    public static Command driveWhileAt90Heading;
    public static Command driveWhileAt180Heading;
    public static Command driveWhileAt270Heading;
    public static Command rotateShoulderToBackdrop;
    public static Command rotateShoulderToIntake;
    public static Command openClaw;
    public static Command closeClaw;
    public static Command liftHome;
    public static Command liftLow;
    public static Command liftMid;
    public static Command liftHigh;
    public static Command wait;
    public static SequentialCommandGroup scorePixel;
    public static ParallelRaceGroup blueBackdropBackup;
    public static ParallelRaceGroup redBackdropBackup;

    private static DriveSubsystem driveSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static EndEffectorSubsystem endEffectorSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static MecanumDriveMona mecanumDrive;

    private static IndicatorLightSubsystem indicatorLightSubsystem;

    private static Command indicatorLightChange;

    public static void MakeRobotDriveBaseCommands() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        mecanumDrive = driveSubsystem.mecanumDrive;
        defaultDriveCommand = new DefaultDriveCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        driveWhileAt0Heading = new DriveWithConstantHeadingCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                0);

        driveWhileAt90Heading = new DriveWithConstantHeadingCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                90);

        driveWhileAt180Heading = new DriveWithConstantHeadingCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                180);

        driveWhileAt270Heading = new DriveWithConstantHeadingCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                270);

        blueBackdropBackup = MakeCancelableBackupFromBlueBackdropCommand();
        redBackdropBackup = MakeCancelableBackupFromRedBackdropCommand();

    }

    public static void MakeRobotVisionCommands() {
        MakeRobotDriveBaseCommands();
    }

    public static void MakeRobotIntakeCommands() {
        intakeSubsystem = Robot.getInstance().getIntakeSubsystem();
    }


    public static void MakeRobotScoringArmCommands() {

//        endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        defaultLiftSlideCommand = new DefaultLiftSlideCommand(liftSlideSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY);

        rotateShoulderToBackdrop = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        rotateShoulderToIntake = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

        liftHome = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        liftLow = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        liftMid = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        liftHigh = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HIGH);

//        openClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.OPEN);
//
//        closeClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.CLOSED);
        wait = new WaitCommand(2000);

//        scorePixel = MakeScorePixelCommand();
    }

    public static void MakeRobotCenterStageCommands() {
        MakeRobotDriveBaseCommands();

    }

    public static void MakeRobotIndicatorLightChangeCommands(){
        indicatorLightSubsystem = Robot.getInstance().getIndicatorLightSubsystem();
        indicatorLightChange = new IndicatorLightChangeCommand(indicatorLightSubsystem);
    }

    private static SequentialCommandGroup MakeScorePixelCommand() {
        scorePixel = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        liftMid,
                        rotateShoulderToBackdrop),
                wait,
                openClaw,
                wait,
                closeClaw,
                wait,
                rotateShoulderToIntake,
                wait,
                liftHome
        );
        return scorePixel;
    }

    private static ParallelRaceGroup MakeCancelableBackupFromBlueBackdropCommand() {
        MakeBackUpFromBlueBackdropAction makeBackUpFromBlueBackdropAction = new MakeBackUpFromBlueBackdropAction();
        ParallelRaceGroup seqCommand = new ParallelRaceGroup(
                new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromBlueBackdropAction.makeAction()),
                new IsGamepadActiveCommand()
        );
        return seqCommand;
    }

    private static ParallelRaceGroup MakeCancelableBackupFromRedBackdropCommand() {
        MakeBackUpFromRedBackdropAction makeBackUpFromRedBackdropAction = new MakeBackUpFromRedBackdropAction();
        ParallelRaceGroup seqCommand = new ParallelRaceGroup(
                new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromRedBackdropAction.makeAction()),
                new IsGamepadActiveCommand()
        );
        return seqCommand;
    }
}
