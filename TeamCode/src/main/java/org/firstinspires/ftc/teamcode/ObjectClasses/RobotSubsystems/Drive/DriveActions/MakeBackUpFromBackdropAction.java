package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_SPIKE_R;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_PIXEL_TRUSS;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.SUPER_RED_NEUTRAL_PIXEL_TRUSS;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

import java.util.Arrays;

public class MakeBackUpFromBackdropAction {
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
                        drive.kinematics.new WheelVelConstraint(50),
                        new AngularVelConstraint(50)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-60, 60);

        if (MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.RED) {
            t = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(SUPER_RED_NEUTRAL_PIXEL_TRUSS), TANGENT_TOWARD_AUDIENCE, overrideVelConstraint, overrideAccelConstraint)
                    .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_AUDIENCE)
                    .turn(Math.toRadians(-83))
                    .build();
        } else {
            t = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_TRUSS), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_AUDIENCE)
                    .turn(Math.toRadians(83))
                    .build();
        }

        return t;
    }
}
