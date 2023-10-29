package org.firstinspires.ftc.teamcode.ObjectClasses.Actions;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.Arrays;

public class CenterstageActions {
    private static Action t;

    private static double xTarget;
    private static double yTarget;
    private static double currentHeading;

    private static MecanumDriveMona drive;
    public static VelConstraint overrideVelConstraint;
    public static AccelConstraint overrideAccelConstraint;
    public static TurnConstraints overrideTurnConstraint;


    public static Action moveToPoint(double x, double y) {
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
