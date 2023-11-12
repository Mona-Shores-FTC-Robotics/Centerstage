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
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.MoveToPointCommand;
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
        EndEffectorSubsystem endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
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

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new ActuateEndEffectorCommand(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.OPEN),
                        new ActuateEndEffectorCommand(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.CLOSED));


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
                .whenPressed(
                        new SequentialCommandGroup(
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON),
                                new ActuateEndEffectorCommand(endEffectorSubsystem, EndEffectorSubsystem.EndEffectorStates.OPEN)
                        ))
                .whenReleased(
                        new SequentialCommandGroup(
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF),
                                new ActuateEndEffectorCommand(endEffectorSubsystem, EndEffectorSubsystem.EndEffectorStates.CLOSED)
                        ));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            Robot.getInstance().getVisionSubsystem().setDeliverHeight(VisionSubsystem.DeliverHeight.MID);}),
                        readyToScorePixel
                ), false);


        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));


        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        releasePixels
                , false);

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
                        new MoveLiftSlideCommand(liftSlideSubsystem,
                                LiftSlideSubsystem.LiftStates.SAFE),
                        new RotateShoulderCommand(shoulderSubsystem,
                                ShoulderSubsystem.ShoulderStates.BACKDROP),
                        new MoveLiftSlideCommand(liftSlideSubsystem,
                                LiftSlideSubsystem.LiftStates.MID)
                );

        releasePixels =
                new SequentialCommandGroup(
                        new ActuateEndEffectorCommand(endEffectorSubsystem,
                                EndEffectorSubsystem.EndEffectorStates.OPEN),
                        new WaitCommand(325),
                        new MoveToPointCommand(Robot.getInstance().getDriveSubsystem(),
                                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x-2,
                                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y
                        ),
                        new ParallelCommandGroup(
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.SAFE),
                                new ActuateEndEffectorCommand(endEffectorSubsystem,
                                        EndEffectorSubsystem.EndEffectorStates.CLOSED),
                                new RotateShoulderCommand(shoulderSubsystem,
                                        ShoulderSubsystem.ShoulderStates.HALFWAY)
                        ),
                        new ParallelCommandGroup(new RotateShoulderCommand(shoulderSubsystem,
                                ShoulderSubsystem.ShoulderStates.INTAKE),
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.HOME)
                        )
                );
    }
}
