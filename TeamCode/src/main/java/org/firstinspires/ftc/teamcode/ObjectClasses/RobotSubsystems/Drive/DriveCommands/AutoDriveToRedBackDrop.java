package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class AutoDriveToRedBackDrop extends CommandBase {

    private DriveSubsystem driveSubsystem;

    private boolean finished;
    private FtcDashboard dash;
    private Canvas canvas;
    private TelemetryPacket telemetryPacket;

    public AutoDriveToRedBackDrop(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        finished=true;
    }

    @Override
    public void execute() {
        telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay().getOperations().addAll(canvas.getOperations());
        finished = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropRed();
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);
        dash.sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public boolean isFinished() {
        if (finished)
        {
            return true;
        } else return false;
    }
}
