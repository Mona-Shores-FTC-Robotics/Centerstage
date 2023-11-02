package org.firstinspires.ftc.teamcode.ObjectClasses;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeading;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromBlueBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromRedBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public abstract class CenterstageCommands {

    public static Command defaultCommand;
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

    public static void MakeRobotDriveBaseCommands() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        mecanumDrive = driveSubsystem.mecanumDrive;
        defaultCommand = new DefaultDrive(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        driveWhileAt0Heading = new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                0);

        driveWhileAt90Heading = new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                90);

        driveWhileAt180Heading = new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                180);

        driveWhileAt270Heading = new DriveWithConstantHeading(driveSubsystem,
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

        rotateShoulderToBackdrop = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        rotateShoulderToIntake = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

        liftHome = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        liftLow = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        liftMid = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        liftHigh = new MoveLiftSlide(liftSlideSubsystem,
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
