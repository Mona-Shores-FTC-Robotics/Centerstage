package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;

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
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromBlueBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActions.MakeBackUpFromRedBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class VisionDriverBindings {
    public static Command defaultDriveCommand;
    public static Command defaultRRFieldCentricCommand;
    public static Command defaultRRFieldCentricCommand2;
    public static Command defaultRRRobotCentricCommand;
    public static Command defaultRRRobotCentricCommand2;
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
        CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultRRFieldCentricCommand2);

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
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        // move to just outside the correct color wing?
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(backupFromBackdropCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand( ()-> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.LEFT);}));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand( ()-> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.CENTER);}));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand( ()-> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);}));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  BACK/OPTIONS BUTTON                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand( ()-> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Mona driving");
                    CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultDriveCommand);}));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON                                        //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand( ()-> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("RR Field Centric Driving");
                    CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultRRFieldCentricCommand);
                    Robot.getInstance().getActiveOpMode().telemetry.addData("current command", CommandScheduler.getInstance().getDefaultCommand(Robot.getInstance().getDriveSubsystem()));
                }));
    }

    private void MakeCommands() {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                GamepadHandling.getDriverGamepad()::getLeftY,
                GamepadHandling.getDriverGamepad()::getLeftX,
                GamepadHandling.getDriverGamepad()::getRightX
        );

        defaultRRRobotCentricCommand2 = Robot.getInstance().getDriveSubsystem().driveRobotCentric(
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
}

