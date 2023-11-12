package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.RED;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeBackUpFromBlueBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeBackUpFromRedBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class CenterstageDriverBindings {
    public static Command defaultDriveCommand;
    public static Command backupFromBackdropCommand;
    public static Command driveAwayFromBackdropWithConstantHeading;

    public CenterstageDriverBindings(GamepadEx gamepad) {
        //Make the commands to use for the bindings
        MakeCommands(gamepad);

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
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(slowModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        // moves straight back and rotates us toward the wing - can be cancelled to easily grab from the neutral stacks instead
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(backupFromBackdropCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.LEFT);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.CENTER);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  BACK/OPTIONS BUTTON                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

        //This button sets a flag so the next time we are at the backdrop it does a full gyro reset
        //The concept here is if we disconnect, this gives us a way to reset our gyro to a known field position
        //Otherwise, we normally only reset our x/y pose based on the apriltag reading and the heading based on the gyro
        // because the gyro is much more accurate than the april tag based heading
        gamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().resetHeading=true;
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON                                        //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Field Centric Driving");
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = true;
                }), new InstantCommand(() -> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Robot Centric Driving");
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = false;
                }));
    }

    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

        backupFromBackdropCommand = new InstantCommand(()->{
            if (MatchConfig.finalAllianceColor == RED) {
                MakeBackUpFromRedBackdropAction makeBackUpFromRedBackdropAction = new MakeBackUpFromRedBackdropAction();
                new ParallelRaceGroup(
                        new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromRedBackdropAction.makeAction()),
                        new IsGamepadActiveCommand(gamepad)

                ).schedule();
            } else {
                MakeBackUpFromBlueBackdropAction makeBackUpFromBlueBackdropAction = new MakeBackUpFromBlueBackdropAction();
                new ParallelRaceGroup(
                        new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromBlueBackdropAction.makeAction()),
                        new IsGamepadActiveCommand(gamepad)
                ).schedule();
            }
        });
    }
}

