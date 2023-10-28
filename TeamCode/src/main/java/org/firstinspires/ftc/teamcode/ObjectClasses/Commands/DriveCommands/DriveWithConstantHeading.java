package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;

import java.util.function.DoubleSupplier;

public class DriveWithConstantHeading extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double lockedHeadingDegrees;

    private TurnPIDController pid;
    private double currentAngle;

    /**
     * Creates a new command to drive with heading locked.
     */
    public DriveWithConstantHeading(DriveSubsystem subsystem,
                                    DoubleSupplier driveInput, DoubleSupplier strafeInput, double headingDegrees) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        lockedHeadingDegrees = headingDegrees;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        pid = new TurnPIDController(lockedHeadingDegrees, 1, 0, 0, 0);
    }

    @Override
    public void execute() {
        currentAngle = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;

        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.setDriveStrafeTurnValues(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), pid.update(currentAngle));
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }
}