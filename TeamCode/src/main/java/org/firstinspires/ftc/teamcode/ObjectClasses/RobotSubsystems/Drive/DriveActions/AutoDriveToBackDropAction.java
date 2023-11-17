package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
@Config
public class AutoDriveToBackDropAction implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean running;
    private VisionSubsystem.DeliverLocation deliverLocation;
    private int count=0;

    public AutoDriveToBackDropAction() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        running=true;
    }

    public AutoDriveToBackDropAction(VisionSubsystem.DeliverLocation delLoc) {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        deliverLocation=delLoc;
        running=true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        //Reset the timer for the loop timer for the AutoDrive Action (which is only used in Auto)
        MatchConfig.loopTimer.reset();

        if (deliverLocation!=null)
        {
            Robot.getInstance().getVisionSubsystem().setDeliverLocation(deliverLocation);
        }

        //Look for April Tags
        Robot.getInstance().getVisionSubsystem().LookForAprilTags();

        //Run the Autodrive code to generate appropriate drive/turn/strafe values for SpeedControl
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropRed();
        } else{
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropBlue();
        }

        //Run Speed Control using the AprilTag autodrive values
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);

        //Update our pose estimate
        Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

        //Reset the robot pose if we are at the backdrop
        if (Robot.getInstance().getVisionSubsystem().resetPoseReady){
            Robot.getInstance().getVisionSubsystem().resetPoseReady=false;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = Robot.getInstance().getVisionSubsystem().resetPose;
        }

        //Fill and send telemetry packet to the Ftc Dashboard
        DashboardTelemetry(telemetryPacket);

        //Standard Driver Station Telemetry
        MatchConfig.LoopDriverStationTelemetry();

        //Count hhe number of times autodrive code says we are at backdrop - it returns true if we are at backdrop
        if (!running) count++;
        //while the action is running return true
       if (count>4) {
           count=0;
           telemetryPacket.addLine("finished autodriving to tag");
           DashboardTelemetry(telemetryPacket);
           return false;}
        else return true;
    }

    private void DashboardTelemetry(TelemetryPacket telemetryPacket) {
        Canvas c = telemetryPacket.fieldOverlay();
        telemetryPacket.put("x", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x);
        telemetryPacket.put("y", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log()));

        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawPoseHistory(c);
        c.setStroke("#3F51B5");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}
