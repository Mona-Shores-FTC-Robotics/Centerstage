package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands.driveWhileAt0Heading;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeMoveToPointAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;

public class VisionDriverBindings {

    public VisionDriverBindings(GamepadEx gamepad) {

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(driveWhileAt0Heading);

        //todo test these three button bindings - having issues with them only running one time
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), MakeTestRoute()));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(),
                        new MakeMoveToPointAction().moveToPoint(25, 25)));

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(driveWhileAt0Heading);
    }

    private Action MakeTestRoute(){
        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        Action testRoute = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .build();
        return testRoute;
    }

}

