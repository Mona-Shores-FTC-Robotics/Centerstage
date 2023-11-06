package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class AutoDriveToBackDrop implements Action {

    private DriveSubsystem driveSubsystem;
    private boolean running;

    public AutoDriveToBackDrop() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getVisionSubsystem().LookForAprilTags();

        if (MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.RED) {
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropRed();
        } else{
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropBlue();
        }
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);

        telemetryPacket.put("April Tag Drive", driveSubsystem.mecanumDrive.aprilTagDrive);
        telemetryPacket.put("April Tag Strafe", driveSubsystem.mecanumDrive.aprilTagStrafe);
        telemetryPacket.put("April Tag Turn", driveSubsystem.mecanumDrive.aprilTagTurn);

        //while the action is running return true
       if (running) {return true;}
        else return false;
    }
}
