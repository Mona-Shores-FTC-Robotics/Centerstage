package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class AutoDriveToBackDrop implements Action {

    private DriveSubsystem driveSubsystem;
    private boolean running;
    private int count=0;
    public AutoDriveToBackDrop() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        running=true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getVisionSubsystem().LookForAprilTags();

        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropRed();
        } else{
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropBlue();
        }
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);

        Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

        if (Robot.getInstance().getVisionSubsystem().resetPoseReady){
            Robot.getInstance().getVisionSubsystem().resetPoseReady=false;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = Robot.getInstance().getVisionSubsystem().resetPose;
//            Robot.getInstance().getDriveSubsystem().mecanumDrive.poseHistory.clear();
        }

        Canvas c = telemetryPacket.fieldOverlay();
        telemetryPacket.put("April Tag Drive", driveSubsystem.mecanumDrive.aprilTagDrive);
        telemetryPacket.put("April Tag Strafe", driveSubsystem.mecanumDrive.aprilTagStrafe);
        telemetryPacket.put("April Tag Turn", driveSubsystem.mecanumDrive.aprilTagTurn);

        telemetryPacket.put("x", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x);
        telemetryPacket.put("y", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log()));

        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);

        if (!running) count++;
        //while the action is running return true
       if (count>10) {
           count=0;
           return false;}
        else return true;
    }
}
