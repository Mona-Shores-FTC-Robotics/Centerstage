package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeMoveToPointAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class MoveToPointCommand extends CommandBase {

    private double xTarget;
    private double yTarget;
    private DriveSubsystem driveSubsystem;

    private boolean running;
    private FtcDashboard dash;
    private Canvas canvas;
    private TelemetryPacket telemetryPacket;
    private Action internalAction;

    public MoveToPointCommand(DriveSubsystem subsystem, double x, double y) {
        driveSubsystem = subsystem;
        xTarget = x;
        yTarget = y;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        running=true;
        MakeMoveToPointAction makeMoveToPointAction = new MakeMoveToPointAction();
        internalAction = makeMoveToPointAction.moveToPoint(xTarget, yTarget);
        internalAction.preview(canvas);
    }

    @Override
    public void execute() {
        telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay().getOperations().addAll(canvas.getOperations());
        running = internalAction.run(telemetryPacket);
        dash.sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public boolean isFinished() {
        if (running)
        {
            return false;
        } else return true;
    }
}
