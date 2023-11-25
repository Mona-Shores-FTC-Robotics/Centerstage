package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
@Config
public class SlowModeCommand extends CommandBase {

    public static double SLOW_DRIVE_FACTOR = .35;
    public static double SLOW_TURN_FACTOR = .5;
    public static double SLOW_STRAFE_FACTOR = .35;


    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;


    private double previousDriveFactor;
    private double previousStrafeFactor;
    private double previousTurnFactor;

    /**
     * Creates a new SlowMode Command.
     */
    public SlowModeCommand(DriveSubsystem subsystem,
                           DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        //save the normal speed factors to reset them after the command is finished
        previousDriveFactor = Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR;
        previousStrafeFactor = Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR;
        previousTurnFactor =Robot.getInstance().getDriveSubsystem().driveParameters.TURN_SPEED_FACTOR;

        //set the slow mode speed factors
        Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR = SLOW_DRIVE_FACTOR;
        Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR = SLOW_STRAFE_FACTOR;
        Robot.getInstance().getDriveSubsystem().driveParameters.TURN_SPEED_FACTOR = SLOW_TURN_FACTOR;

        //set the apriltag driving override so we aren't limited by the apriltag driving distance
        Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(true);
        driveSubsystem.currentState = DriveSubsystem.DriveStates.SLOW_MANUAL_DRIVE;
    }

    @Override
    public void execute() {
        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.setDriveStrafeTurnValues(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );

        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }

    @Override
    public void end(boolean interrupted) {
        //set the speed factors back to normal
        Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR = previousDriveFactor;
        Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR = previousStrafeFactor;
        Robot.getInstance().getDriveSubsystem().driveParameters.TURN_SPEED_FACTOR = previousTurnFactor;
        Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(false);
    }
}