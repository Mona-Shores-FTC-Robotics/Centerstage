package org.firstinspires.ftc.teamcode.ObjectClasses.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DriveWithConstantHeading;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.ActuateEndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public abstract class Commands {

    public static Command defaultCommand;
    public static Command turnTo0;
    public static Command turnTo90;
    public static Command turnTo180;
    public static Command turnTo270;

    public static Command driveLockedTo90;

    public static SequentialCommandGroup scorePixel;

    public static void MakeTeleOpCommands(){

        DriveSubsystem driveSubsystem = Robot.getInstance().getDriveSubsystem();

        defaultCommand = new DefaultDrive(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        turnTo0 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                0));

        turnTo90 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                90));

        turnTo180 =  new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                180));

        turnTo270 = new InstantCommand(() -> new DriveWithConstantHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                270));

        ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        EndEffectorSubsystem endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        Command rotateBackdrop = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        Command rotateIntake = new RotateShoulder(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

        Command openClaw =  new ActuateEndEffector(endEffectorSubsystem,
                EndEffectorSubsystem.EndEffectorStates.OPEN);

        Command closeClaw = new ActuateEndEffector(endEffectorSubsystem,
                EndEffectorSubsystem.EndEffectorStates.CLOSED);

        Command liftHome = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        Command liftLow = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        Command liftMid = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        Command liftHigh = new MoveLiftSlide(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HIGH);

        Command wait = new WaitCommand(2000);

        scorePixel = new SequentialCommandGroup(
                liftMid,
                wait,
                rotateBackdrop,
                wait,
                openClaw,
                wait,
                closeClaw,
                wait,
                rotateIntake,
                wait,
                liftHome
        );
    }

}
