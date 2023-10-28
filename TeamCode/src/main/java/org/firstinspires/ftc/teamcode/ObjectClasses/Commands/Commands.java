package org.firstinspires.ftc.teamcode.ObjectClasses.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DefaultDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.DriveWithLockedHeading;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.PIDTurn;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

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

        turnTo0 = new PIDTurn(0, .5,0,0,0);
        turnTo90 = new PIDTurn(90, .5,0,0,0);
        turnTo180 = new PIDTurn(180, .5,0,0,0);
        turnTo270 = new PIDTurn(270, .5,0,0,0);

        driveLockedTo90 = new DriveWithLockedHeading(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                90);
    }

}
