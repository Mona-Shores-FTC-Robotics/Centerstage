package org.firstinspires.ftc.teamcode.ObjectClasses.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DriveWithConstantHeading;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public abstract class Commands {

    public static Command defaultCommand;
    public static Command turnTo0;
    public static Command turnTo90;
    public static Command turnTo180;
    public static Command turnTo270;

    public static Command driveLockedTo90;

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
    }

}
