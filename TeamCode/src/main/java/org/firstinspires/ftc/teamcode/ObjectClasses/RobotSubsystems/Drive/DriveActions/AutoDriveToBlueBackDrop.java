package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class AutoDriveToBlueBackDrop implements Action {

    private DriveSubsystem driveSubsystem;

    private boolean finished;
    private int counter;

    public AutoDriveToBlueBackDrop() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        counter=0;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getVisionSubsystem().LookForAprilTags();
        finished = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropBlue();
        if (finished) counter++;
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);

        telemetryPacket.put("April Tag Drive", driveSubsystem.mecanumDrive.aprilTagDrive);
        telemetryPacket.put("April Tag Strafe", driveSubsystem.mecanumDrive.aprilTagStrafe);
        telemetryPacket.put("April Tag Turn", driveSubsystem.mecanumDrive.aprilTagTurn);

       if (counter>10) {return false;}
        else return true;
    }
}
