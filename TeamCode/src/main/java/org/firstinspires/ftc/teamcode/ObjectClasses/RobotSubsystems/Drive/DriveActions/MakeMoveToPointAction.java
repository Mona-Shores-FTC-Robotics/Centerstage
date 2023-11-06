package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.Arrays;

public class MakeMoveToPointAction {
    private Action t;

    private double xTarget;
    private double yTarget;
    private double currentHeading;

    private MecanumDriveMona drive;
    public VelConstraint overrideVelConstraint;
    public AccelConstraint overrideAccelConstraint;
    public TurnConstraints overrideTurnConstraint;

    public Action moveToPoint(double x, double y) {
        xTarget = x;
        yTarget = y;
        currentHeading = Robot.getInstance().getGyroSubsystem().getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(5),
                        new AngularVelConstraint(5)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-10, 10);

        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(5), -Math.toRadians(5), Math.toRadians(5));

        t = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d( xTarget, yTarget), currentHeading)
                .build();

        return t;
    }
}
