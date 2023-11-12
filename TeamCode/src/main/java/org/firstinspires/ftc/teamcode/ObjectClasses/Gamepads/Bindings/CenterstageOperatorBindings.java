package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateEndEffectorCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakePowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class CenterstageOperatorBindings {

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;
    public static SequentialCommandGroup readyToScorePixel;
    public static SequentialCommandGroup releasePixels;

    public CenterstageOperatorBindings(GamepadEx operatorGamepad) {

        MakeCombinationCommands();

        VisionSubsystem visionSubsystem = Robot.getInstance().getVisionSubsystem();
        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();
        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        //do nothing

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER                                         //
        //                                                      //
        //////////////////////////////////////////////////////////

        //end game?

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        //end game?

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

       // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON))
                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            Robot.getInstance().getVisionSubsystem().setDeliverHeight(VisionSubsystem.DeliverHeight.HIGH);}),
                        readyToScorePixel
                ), false)
                .whenReleased(releasePixels);

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        //set lift height to mid

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        //Set lift height to low?


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

    private void MakeCombinationCommands() {

        EndEffectorSubsystem endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        readyToScorePixel =
                new SequentialCommandGroup(
                        new ActuateEndEffectorCommand(endEffectorSubsystem,
                                EndEffectorSubsystem.EndEffectorStates.CLOSED),
                        new ParallelCommandGroup(
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.MID),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new RotateShoulderCommand(shoulderSubsystem,
                                                ShoulderSubsystem.ShoulderStates.BACKDROP)
                                )
                        )
                );

        releasePixels =
                new SequentialCommandGroup(
                        new ActuateEndEffectorCommand(endEffectorSubsystem,
                                EndEffectorSubsystem.EndEffectorStates.OPEN),
                        new WaitCommand(325),
                        new ParallelCommandGroup(
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.SAFE ),
                                new ActuateEndEffectorCommand(endEffectorSubsystem,
                                        EndEffectorSubsystem.EndEffectorStates.CLOSED),
                                new RotateShoulderCommand(shoulderSubsystem,
                                        ShoulderSubsystem.ShoulderStates.HALFWAY)
                        ),
                        new ParallelCommandGroup(new RotateShoulderCommand(shoulderSubsystem,
                                ShoulderSubsystem.ShoulderStates.INTAKE),
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.HOME)
                        ),
                        new ActuateEndEffectorCommand(endEffectorSubsystem,
                                EndEffectorSubsystem.EndEffectorStates.OPEN)
                );
    }

}
