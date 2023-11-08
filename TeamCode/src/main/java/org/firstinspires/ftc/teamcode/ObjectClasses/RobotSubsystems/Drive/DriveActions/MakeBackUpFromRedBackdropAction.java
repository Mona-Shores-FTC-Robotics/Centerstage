package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.HALF_TILE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TILE;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.Arrays;

public class MakeBackUpFromRedBackdropAction {
    private Action t;

    private double currentHeading;

    private MecanumDriveMona drive;
    public VelConstraint overrideVelConstraint;
    public AccelConstraint overrideAccelConstraint;
    public TurnConstraints overrideTurnConstraint;

    public Action makeAction() {
        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        currentHeading = Robot.getInstance().getGyroSubsystem().getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        overrideVelConstraint =
//                new MinVelConstraint(Arrays.asList(
//                        drive.kinematics.new WheelVelConstraint(5),
//                        new AngularVelConstraint(5)
//                ));
//
//        overrideAccelConstraint = new ProfileAccelConstraint(-10, 10);
//
//        overrideTurnConstraint = new TurnConstraints(
//                Math.toRadians(5), -Math.toRadians(5), Math.toRadians(5));

        t = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d( drive.pose.position.x-TILE*3-HALF_TILE, drive.pose.position.y))
                .turn(Math.toRadians(-90))
                .build();
        return t;
    }
}
