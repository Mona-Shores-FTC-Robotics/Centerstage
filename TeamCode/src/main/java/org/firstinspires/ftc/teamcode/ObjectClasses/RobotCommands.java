package org.firstinspires.ftc.teamcode.ObjectClasses;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.DefaultLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public abstract class RobotCommands {
    public static Command defaultLiftSlideCommand;

    public static Command rotateShoulderToBackdrop;
    public static Command rotateShoulderToIntake;
    public static Command openClaw;
    public static Command closeClaw;
    public static Command liftHome;
    public static Command liftLow;
    public static Command liftMid;
    public static Command liftHigh;
    public static Command wait;
    public static SequentialCommandGroup scorePixel;
    public static ParallelRaceGroup blueBackdropBackup;
    public static ParallelRaceGroup redBackdropBackup;

    private static DriveSubsystem driveSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static EndEffectorSubsystem endEffectorSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static MecanumDriveMona mecanumDrive;


    public static void MakeRobotScoringArmCommands() {

//        endEffectorSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        defaultLiftSlideCommand = new DefaultLiftSlideCommand(liftSlideSubsystem,
                GamepadHandling.getDriverGamepad()::getLeftY);

        rotateShoulderToBackdrop = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        rotateShoulderToIntake = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

        liftHome = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        liftLow = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        liftMid = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        liftHigh = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HIGH);

//        openClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.OPEN);
//
//        closeClaw = new ActuateEndEffector(endEffectorSubsystem,
//                EndEffectorSubsystem.EndEffectorStates.CLOSED);
        wait = new WaitCommand(2000);

//        scorePixel = MakeScorePixelCommand();
    }

    private static SequentialCommandGroup MakeScorePixelCommand() {
        scorePixel = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        liftMid,
                        rotateShoulderToBackdrop),
                wait,
                openClaw,
                wait,
                closeClaw,
                wait,
                rotateShoulderToIntake,
                wait,
                liftHome
        );
        return scorePixel;
    }
}
