package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeMoveToPointAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;

public class VisionDriverBindings {
    public static Command defaultDriveCommand;
    public static Command driveWhileAt0Heading;
    private static DriveSubsystem driveSubsystem;

    public VisionDriverBindings(GamepadEx gamepad) {
        //Make the commands to use for the bindings
        MakeCommands();

        // LEFT STICK / RIGHT STICK - Normal Driving
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, defaultDriveCommand);

        //RIGHT BUMPER - While held down, drive normally but hold camera heading toward backdrop
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(driveWhileAt0Heading);

        //X BUTTON
        //backup through the truss right behind the backdrop to see the other april tag?

        //Y BUTTON
        // move to just outside the correct color wing?

        //todo test these three button bindings - having issues with them only running one time
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), MakeTestRoute()));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(),
                        new MakeMoveToPointAction().moveToPoint(25, 25)));

    }

    private void MakeCommands() {
        defaultDriveCommand = new DefaultDriveCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        driveWhileAt0Heading = new DriveWithConstantHeadingCommand(driveSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                Math.toDegrees(FACE_TOWARD_BACKSTAGE));
    }

    private Action MakeTestRoute(){
        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        Action testRoute = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .build();
        return testRoute;
    }

}

