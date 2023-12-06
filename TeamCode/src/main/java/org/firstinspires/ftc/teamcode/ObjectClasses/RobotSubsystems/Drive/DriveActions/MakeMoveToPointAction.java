package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MakeMoveToPointAction {
    private Action t;

    private double xTarget;
    private double yTarget;
    private double currentHeading;

    private MecanumDrive drive;
    public VelConstraint overrideVelConstraint;
    public AccelConstraint overrideAccelConstraint;
    public TurnConstraints overrideTurnConstraint;

    public Action moveToPoint(double x, double y) {
        xTarget = x;
        yTarget = y;
        currentHeading = Robot.getInstance().getGyroSubsystem().currentRelativeYawRadians;

        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        t = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d( xTarget, yTarget), currentHeading)
                .build();

        return t;
    }
}
