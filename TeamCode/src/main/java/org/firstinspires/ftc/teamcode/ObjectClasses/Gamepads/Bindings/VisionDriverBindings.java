package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo0;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo180;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo270;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo90;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Actions.CenterstageActions;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.ActionAsCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.MoveToPoint;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class VisionDriverBindings {
    private Action turn270;
    private Action turn90;

    public VisionDriverBindings(GamepadEx gamepad) {


        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(turnTo0);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(turnTo180);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(turnTo90);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(turnTo270);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(turnTo0);

        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        Action selectedRoute = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .build();

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new ActionAsCommand(Robot.getInstance().getDriveSubsystem(), selectedRoute));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new ActionAsCommand(Robot.getInstance().getDriveSubsystem(), CenterstageActions.moveToPoint(25, 25)));


//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(()->{
//                    if (GamepadHandling.driverGamepadIsActive()) CommandScheduler.getInstance().cancelAll();
//                }));

    }
}

