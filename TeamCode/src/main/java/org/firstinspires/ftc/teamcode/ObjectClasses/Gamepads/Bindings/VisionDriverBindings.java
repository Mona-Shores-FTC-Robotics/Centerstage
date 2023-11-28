package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.RED;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.TurnToAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeBackUpFromBackdropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveXInchesPID;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.SlowModeCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.StrafeXInchesPID;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReleaseDroneCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class VisionDriverBindings {
    public Command defaultDriveCommand;
    public Command slowModeCommand;
    public Command backupFromBackdropCommand;

    public VisionDriverBindings(GamepadEx gamepad) {
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
                .whenHeld(slowModeCommand, true);

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
        //  START BUTTON  - FIELD CENTRIC DRIVING               //
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

        //////////////////////////////////////////////////////////
        //                                                      //
        //  DPAD UP  - Turn to 0 Degrees                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), new TurnToAction(0)));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  DPAD LEFT - Move 6 inches to the left               //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new StrafeXInchesPID(Robot.getInstance().getDriveSubsystem(), 12));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  DPAD RIGHT - Move 6 inches to the right             //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new StrafeXInchesPID(Robot.getInstance().getDriveSubsystem(), -12));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  DPAD DOWN  - Backup 6 inches                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new DriveXInchesPID(Robot.getInstance().getDriveSubsystem(), 12));


        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT TRIGGER                                         //
        //                                                      //
        //////////////////////////////////////////////////////////

        TriggerReader leftTriggerReader = new TriggerReader(
                gamepad, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        Trigger leftTrigger  = new Trigger(leftTriggerReader::wasJustPressed);
        leftTrigger.toggleWhenActive(
                new InstantCommand(()->{Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(false);}),
                new InstantCommand(()->{Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(true);}));

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT TRIGGER                                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        //This should require the user to hold the trigger down for 1 second before the drone fires
        TriggerReader rightTriggerReader = new TriggerReader(
                gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        Trigger rightTrigger  = new Trigger(rightTriggerReader::isDown);
        rightTrigger.whileActiveOnce(new ParallelCommandGroup(
            new WaitCommand(1000),
            new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.FLY)));
    }

    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

        slowModeCommand = new SlowModeCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

        backupFromBackdropCommand = new InstantCommand(()->{
            if (MatchConfig.finalAllianceColor == RED) {
                MakeBackUpFromBackdropAction makeBackUpFromBackdropAction = new MakeBackUpFromBackdropAction();
                new ParallelRaceGroup(
                        new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromBackdropAction.makeAction()),
                        new IsGamepadActiveCommand(gamepad)

                ).schedule();
            } else {
                MakeBackUpFromBackdropAction makeBackUpFromBackdropAction = new MakeBackUpFromBackdropAction();
                new ParallelRaceGroup(
                        new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), makeBackUpFromBackdropAction.makeAction()),
                        new IsGamepadActiveCommand(gamepad)
                ).schedule();
            }
        });
    }
}

