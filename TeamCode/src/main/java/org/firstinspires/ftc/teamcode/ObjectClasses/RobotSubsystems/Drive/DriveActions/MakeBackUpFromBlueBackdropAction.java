package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_SPIKE_R_LINE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_TRUSS;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.Arrays;

public class MakeBackUpFromBlueBackdropAction {
    private Action t;
    private double currentHeading;

    private MecanumDriveMona drive;
    public VelConstraint overrideVelConstraint;
    public AccelConstraint overrideAccelConstraint;
    public TurnConstraints overrideTurnConstraint;

    public Action makeAction() {
        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        currentHeading = Robot.getInstance().getGyroSubsystem().currentRelativeYawRadians;

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(5),
                        new AngularVelConstraint(5)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-10, 10);

        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(5), -Math.toRadians(5), Math.toRadians(5));

        t = drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d( drive.pose.position.x-TILE*3-HALF_TILE, drive.pose.position.y))
                .setReversed(true)
                .splineToLinearHeading(BLUE_TRUSS, TANGENT_TOWARD_AUDIENCE)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_SPIKE_R_LINE), TANGENT_TOWARD_AUDIENCE)
                .turn(Math.toRadians(83))
                .build();
        return t;
    }
}
