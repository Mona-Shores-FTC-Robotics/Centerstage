package org.firstinspires.ftc.teamcode.ObjectClasses.Utility;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class TelemetryAction implements Action {

    public TelemetryAction() {

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        telemetryPacket.put("x", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x);
        telemetryPacket.put("y", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log()));

        Robot.getInstance().getActiveOpMode().telemetry.addData("x", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x);
        Robot.getInstance().getActiveOpMode().telemetry.addData("y", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y);
        Robot.getInstance().getActiveOpMode().telemetry.addData("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log()));

        return false;
    }
}