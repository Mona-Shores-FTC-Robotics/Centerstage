package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command will run on.

     */
    public DefaultDrive(DriveSubsystem subsystem,
                        DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        drive = driveInput;
        strafe = strafeInput;
        turn = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
            driveSubsystem.mecanumDriveSpeedControl(drive.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
    }


}