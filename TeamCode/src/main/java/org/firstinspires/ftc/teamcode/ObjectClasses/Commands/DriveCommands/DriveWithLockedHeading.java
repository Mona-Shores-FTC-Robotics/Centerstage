package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import kotlin.random.Random;

public class DriveWithLockedHeading extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double lockedHeadingDegrees;


    /**
     * Creates a new command to drive with heading locked.
     */
    public DriveWithLockedHeading(DriveSubsystem subsystem,
                                  DoubleSupplier driveInput, DoubleSupplier strafeInput, double headingDegrees) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        lockedHeadingDegrees = headingDegrees;
    }

    @Override
    public void initialize() {
        DoubleSupplier zeroSupplier = () -> 0.0;

        ParallelCommandGroup lockedDriving = new ParallelCommandGroup(
                new DefaultDrive(driveSubsystem, driveSupplier, strafeSupplier, zeroSupplier),
                new PIDTurn(lockedHeadingDegrees, 5, 0,0,0));

        lockedDriving.schedule();
    }
}