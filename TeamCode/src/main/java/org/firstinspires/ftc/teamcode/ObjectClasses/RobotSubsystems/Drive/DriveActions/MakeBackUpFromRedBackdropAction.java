package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_PIXEL_WING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.HALF_TILE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_NEUTRAL_PIXEL_WING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_SPIKE_L_LINE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_TRUSS;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TILE;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

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
        currentHeading = Robot.getInstance().getGyroSubsystem().currentRelativeYawRadians;

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(100),
                        new AngularVelConstraint(100)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-150, 150);

//        overrideTurnConstraint = new TurnConstraints(
//                Math.toRadians(5), -Math.toRadians(5), Math.toRadians(5));

        t = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_TRUSS), TANGENT_TOWARD_AUDIENCE)
                .splineToConstantHeading(PoseToVector(RED_SPIKE_L_LINE), TANGENT_TOWARD_AUDIENCE)
                .turn(Math.toRadians(-83))
                .build();
        return t;
    }
}
