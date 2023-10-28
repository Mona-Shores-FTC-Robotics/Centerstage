package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveWithLockedHeading extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final double turn;

    /**
     * Creates a new DefaultDrive.

     */
    public DriveWithLockedHeading(DriveSubsystem subsystem,
                                  DoubleSupplier driveInput, DoubleSupplier strafeInput, double lockedHeading) {
        driveSubsystem = subsystem;
        drive = driveInput;
        strafe = strafeInput;
        turn = lockedHeading;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(drive.getAsDouble(), strafe.getAsDouble(), turn);
    }
}