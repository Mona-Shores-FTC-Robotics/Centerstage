package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Roadrunner.MecanumDrive;

import java.util.Arrays;

public class MakeLineToXAction {
    private Action t;

    private double xTarget;

    private MecanumDrive drive;
    public VelConstraint overrideVelConstraint;
    public AccelConstraint overrideAccelConstraint;


    public Action lineToX(double x) {
        xTarget = x;
        drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(25),
                        new AngularVelConstraint(25)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-25, 25);

        t = drive.actionBuilder(drive.pose)
                .lineToX(xTarget, overrideVelConstraint, overrideAccelConstraint)
                .build();

        return t;
    }
}
