package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromBlueBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromRedBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeMoveToPointAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class VisionDriverBindings {
    public static Command defaultDriveCommand;
    public static Command driveWhileAt0Heading;
    public static Command backupFromBackdropCommand;

    public VisionDriverBindings(GamepadEx gamepad) {
        //Make the commands to use for the bindings
        MakeCommands();

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //                                                      //
        //////////////////////////////////////////////////////////
        CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultDriveCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER                                         //
        //                                                      //
        //////////////////////////////////////////////////////////

        //While held down, drive normally but hold camera heading toward backdrop
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(driveWhileAt0Heading);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        // move to just outside the correct color wing?
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(backupFromBackdropCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        //todo test - having issues with them only running one time
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), MakeTestRoute()));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(),
                        new MakeMoveToPointAction().moveToPoint(25, 25)));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  LEFT TRIGGER                                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  RIGHT TRIGGER                                       //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  OPTIONS BUTTON                                      //
        //                                                      //
        //////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON                                        //
        //                                                      //
        //////////////////////////////////////////////////////////

    }

    private void MakeCommands() {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        driveWhileAt0Heading = new DriveWithConstantHeadingCommand(Robot.getInstance().getDriveSubsystem(),
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                Math.toDegrees(FACE_TOWARD_BACKSTAGE));

        backupFromBackdropCommand = new InstantCommand(()->{
            if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
               MakeBackUpFromRedBackdropAction makeBackUpFromRedBackdropAction = new MakeBackUpFromRedBackdropAction();
               new ParallelRaceGroup(
                            new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromRedBackdropAction.makeAction()),
                            new IsGamepadActiveCommand()
                    ).schedule();
            } else {
                MakeBackUpFromBlueBackdropAction makeBackUpFromBlueBackdropAction = new MakeBackUpFromBlueBackdropAction();
                new ParallelRaceGroup(
                        new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromBlueBackdropAction.makeAction()),
                        new IsGamepadActiveCommand()
                ).schedule();
            }
        });
    }

    private Action MakeTestRoute(){
        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        Action testRoute = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .build();
        return testRoute;
    }

}

