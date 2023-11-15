package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeLineToXAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.MakeMoveToPointAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class LineToXRelativeAction implements Action {

    private double xTarget;
    private double yTarget;
    private DriveSubsystem driveSubsystem;

    private boolean running;
    private FtcDashboard dash;
    private Canvas canvas;
    private TelemetryPacket telemetryPacket;
    private Action internalAction;
    private boolean hasNotInit=true;

    public LineToXRelativeAction(double x) {
        xTarget = x;
    }

    public void init() {
        hasNotInit=false;
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        running=true;
        MakeLineToXAction makeLineToXAction = new MakeLineToXAction();
        internalAction = makeLineToXAction.lineToX(
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x + xTarget);
        internalAction.preview(canvas);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) init();
        telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay().getOperations().addAll(canvas.getOperations());
        running = internalAction.run(telemetryPacket);
        dash.sendTelemetryPacket(telemetryPacket);

        if (running)
        {
            return false;
        } else return true;
    }
}
