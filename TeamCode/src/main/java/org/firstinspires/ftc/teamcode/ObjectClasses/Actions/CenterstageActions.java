package org.firstinspires.ftc.teamcode.ObjectClasses.Actions;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class CenterstageActions {
    private static Action t;

    private static double xTarget;
    private static double yTarget;
    private static double currentHeading;

    private static MecanumDriveMona drive;

    public static Action moveToPoint(double x, double y){
        xTarget = x;

        yTarget = y;
        currentHeading = Robot.getInstance().getGyroSubsystem().getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        t = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d( xTarget, yTarget), currentHeading)
                .build();
        return t;
    }

}
