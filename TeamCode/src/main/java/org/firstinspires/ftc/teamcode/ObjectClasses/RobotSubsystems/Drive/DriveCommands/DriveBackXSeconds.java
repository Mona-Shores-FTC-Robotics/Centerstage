package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

/**
 * A command to drive the robot with joystick input *
 */
public class DriveBackXSeconds extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private ElapsedTime timer;
    private MecanumDriveMona mecanumDrive;
    private double time;
    /**
     * Creates a new DefaultDrive.
     */
    public DriveBackXSeconds(DriveSubsystem subsystem, double timeToDrive) {
        driveSubsystem = subsystem;
        time = timeToDrive;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        driveSubsystem.currentState = DriveSubsystem.DriveStates.BACKUP_FROM_BACKDROP;
    }

    @Override
    public void execute() {
        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(-.5, 0, 0);
    }

    @Override
    public boolean isFinished(){
        if (timer.seconds() > time) {
            driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(0, 0, 0);
            return true;
        } else return false;
    }
}