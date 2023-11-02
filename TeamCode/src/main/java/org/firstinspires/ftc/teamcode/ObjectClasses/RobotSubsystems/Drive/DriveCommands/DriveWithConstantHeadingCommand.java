package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;

import java.util.function.DoubleSupplier;

public class DriveWithConstantHeadingCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double lockedHeadingDegrees;

    private FtcDashboard dash;
    private Canvas c;
    private TelemetryPacket p;

    private TurnPIDController pid;
    private double currentAngle;
    private MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

    /**
     * Creates a new command to drive with heading locked.
     */
    public DriveWithConstantHeadingCommand(DriveSubsystem subsystem,
                                           DoubleSupplier driveInput, DoubleSupplier strafeInput, double headingDegrees) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        lockedHeadingDegrees = headingDegrees;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        pid = new TurnPIDController(lockedHeadingDegrees, .1, 0, 0, 0);
        dash = FtcDashboard.getInstance();
        c = new Canvas();
    }

    @Override
    public void execute() {
        p = new TelemetryPacket();
        p.fieldOverlay().getOperations().addAll(c.getOperations());

        currentAngle = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;

        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.setDriveStrafeTurnValues(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), pid.update(currentAngle));
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);

        p.put("x", mecanumDrive.pose.position.x);
        p.put("y", mecanumDrive.pose.position.y);
        p.put("heading (deg)", Math.toDegrees(mecanumDrive.pose.heading.log()));

        Canvas c = p.fieldOverlay();
        mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        mecanumDrive.drawRobot(c, mecanumDrive.pose);
        dash.sendTelemetryPacket(p);
    }
}