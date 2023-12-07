package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;

import java.util.function.DoubleSupplier;

@Config
public class SlowModeConstantHeadingCommand extends CommandBase {

    public static double P_TERM = .016;
    public static double I_TERM = 0;
    public static double D_TERM = 0;
    public static double F_TERM = .15;

    public static double SLOW_DRIVE_FACTOR = .35;
    public static double SLOW_STRAFE_FACTOR = .5;

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double lockedHeadingDegrees;

    private TurnPIDController pid;
    private double currentAngle;

    private double previousDriveFactor;
    private double previousStrafeFactor;

    /**
     * Creates a new SlowMode Command that holds the robot heading at zero (toward the backdrop)
     */
    public SlowModeConstantHeadingCommand(DriveSubsystem subsystem,
                                          DoubleSupplier driveInput, DoubleSupplier strafeInput, double relativeHeadingDegrees) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        lockedHeadingDegrees = relativeHeadingDegrees;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        pid = new TurnPIDController(lockedHeadingDegrees, P_TERM, I_TERM, D_TERM, F_TERM);

        //save the normal speed factors to reset them after the command is finished
        previousDriveFactor = Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR;
        previousStrafeFactor = Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR;

        //set the slow mode speed factors
        Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR = SLOW_DRIVE_FACTOR;
        Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR = SLOW_STRAFE_FACTOR;

        //set the apriltag driving override so we aren't limited by the apriltag driving distance
        Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(true);
    }

    @Override
    public void execute() {
        currentAngle = Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees;

        driveSubsystem.setDriveStrafeTurnValues(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                pid.update(currentAngle)
        );

        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }

    @Override
    public void end(boolean interrupted) {
        //set the speed factors back to normal
        Robot.getInstance().getDriveSubsystem().driveParameters.DRIVE_SPEED_FACTOR = previousDriveFactor;
        Robot.getInstance().getDriveSubsystem().driveParameters.STRAFE_SPEED_FACTOR = previousStrafeFactor;
        Robot.getInstance().getDriveSubsystem().setOverrideAprilTagDriving(false);
    }

}