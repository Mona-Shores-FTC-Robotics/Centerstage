package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
public class DefaultDriveFieldCentricCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    private FtcDashboard dash;
    private Canvas c;
    private TelemetryPacket p;

    private MecanumDriveMona mecanumDrive;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command will run on.
     */
    public DefaultDriveFieldCentricCommand(DriveSubsystem subsystem,
                                           DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        dash = FtcDashboard.getInstance();
        c = new Canvas();
    }

    @Override
    public void execute() {
        p = new TelemetryPacket();
        p.fieldOverlay().getOperations().addAll(c.getOperations());

        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
//        driveSubsystem.setDriveStrafeTurnValues(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), turnSupplier.getAsDouble());
        driveSubsystem.fieldOrientedControl(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble());
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