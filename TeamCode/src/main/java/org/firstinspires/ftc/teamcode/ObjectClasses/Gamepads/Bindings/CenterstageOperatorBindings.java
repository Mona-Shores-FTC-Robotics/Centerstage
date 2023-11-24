package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateGripperCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.DefaultLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ChangeWinchPowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.MoveClimberArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.WinchHoldPositionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakePowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class CenterstageOperatorBindings {

    public CenterstageOperatorBindings(GamepadEx operatorGamepad) {
        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();
        GripperSubsystem gripperSubsystem = Robot.getInstance().getGripperSubsystem();
        ClimberSubsystem climberSubsystem = Robot.getInstance().getClimberSubsystem();

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        //Left Stick - Move Lift up and down
        CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getLiftSlideSubsystem(),
                new DefaultLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), operatorGamepad::getLeftY));

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - ROBOT UP WITH WINCH                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new InstantCommand(() -> {
                                    //todo should the climber arm be put down when we climb anymore? (took it out for now)
                                    new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_UP).schedule();})
                )
                .whenReleased(
                        new WinchHoldPositionCommand(climberSubsystem)
                );

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP - Max Delivery Height                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()->{
                    Robot.getInstance().getLiftSlideSubsystem().setDeliverHeight(LiftSlideSubsystem.LiftStates.MAX);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-RIGHT - High Delivery Height                    //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()->
                    Robot.getInstance().getLiftSlideSubsystem().setDeliverHeight(LiftSlideSubsystem.LiftStates.HIGH)));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-LEFT - Mid Delivery Height                      //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()->
                    Robot.getInstance().getLiftSlideSubsystem().setDeliverHeight(LiftSlideSubsystem.LiftStates.MID)));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN - Low Delivery Height                      //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()->
                    Robot.getInstance().getLiftSlideSubsystem().setDeliverHeight(LiftSlideSubsystem.LiftStates.LOW)));

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER  - CLIMBER ARM TO READY POSITION         //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> {
                            new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.READY).schedule();
                            Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(true);
                        }),
                        new InstantCommand(() -> {
                            new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED).schedule();
                            Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(false);
                        }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON - INTAKE                                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.OPEN),
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON, IntakeSubsystem.IntakeStates.INTAKE_SLOW)
                        ))
                .whenReleased(
                        new SequentialCommandGroup(
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF),
                                new WaitCommand(300),
                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.CLOSED)
                        ));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - REVERSE INTAKE                           //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON - Extend Lift                              //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->
                            new MakeOperatorCombinationCommands().ReadyToScorePixelCommand().schedule()));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - Release Pixels / Retract Arm             //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()-> {
                            if (Robot.getInstance().getLiftSlideSubsystem().isStageOneReleasePixels()) {
                                new SequentialCommandGroup(
                                        new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.OPEN),
                                        new WaitCommand(500),
                                        new InstantCommand(()->Robot.getInstance().getLiftSlideSubsystem().setStageOneReleasePixels(false)))
                                        .schedule();
                            } else {
                               Robot.getInstance().getLiftSlideSubsystem().setStageOneReleasePixels(true);
                               new MakeOperatorCombinationCommands().PutArmAway().schedule(false);
                            }
                        }));


        //////////////////////////////////////////////////////////
        //                                                      //
        //  LEFT TRIGGER - Release Pixel                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        TriggerReader leftTriggerReader = new TriggerReader(
                operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        Trigger leftTrigger = new Trigger(leftTriggerReader::wasJustPressed);
        leftTrigger.toggleWhenActive(
                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.ONE_PIXEL_RELEASE_POSITION),
                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.CLOSED)
        );

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

    private class MakeOperatorCombinationCommands{

        GripperSubsystem gripperSubsystem = Robot.getInstance().getGripperSubsystem();
        ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        private Command ReadyToScorePixelCommand() {
            GripperSubsystem gripperSubsystem = Robot.getInstance().getGripperSubsystem();
            ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
            LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();
            return new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new ActuateGripperCommand(gripperSubsystem,
                                    GripperSubsystem.GripperStates.CLOSED),
                            new RotateShoulderCommand(shoulderSubsystem,
                                    ShoulderSubsystem.ShoulderStates.BACKDROP),
                            new WaitCommand(150),
                            new MoveLiftSlideCommand(liftSlideSubsystem, Robot.getInstance().getLiftSlideSubsystem().getDeliverHeight()))
                    );
        }

        private Command PutArmAway() {
            return new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new RotateShoulderCommand(shoulderSubsystem,
                                            ShoulderSubsystem.ShoulderStates.HALFWAY),
                                    new ActuateGripperCommand(gripperSubsystem,
                                            GripperSubsystem.GripperStates.CLOSED),
                                    new MoveLiftSlideCommand(liftSlideSubsystem, LiftSlideSubsystem.LiftStates.SAFE)
                            ),
                            new WaitCommand(250),
                            new RotateShoulderCommand(shoulderSubsystem,
                                    ShoulderSubsystem.ShoulderStates.INTAKE),
                            new WaitCommand(250),
                            new MoveLiftSlideCommand(liftSlideSubsystem,
                                    LiftSlideSubsystem.LiftStates.HOME)
                    );
        }
    }
}
