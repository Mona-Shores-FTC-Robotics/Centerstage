package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
    public final MecanumDriveMona mecanumDrive;

    public DriveSubsystem(HardwareMap hardwareMap) {
        mecanumDrive = new MecanumDriveMona(hardwareMap, new Pose2d(0,0,0));
    }

    public void init()
    {
        mecanumDrive.();
    }



    public Command driveRobotCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return new RunCommand(
                () -> mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                            -leftY.getAsDouble(),
                            -leftX.getAsDouble()
                    ),
                    -rightX.getAsDouble()
            )), this
        );
    }

    public Command driveFieldCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {

        Vector2d input = new Vector2d(
                -leftY.getAsDouble(),
                -leftX.getAsDouble()
        );

        Vector2d rotated = mecanumDrive.pose.heading.inverse().times(new Vector2d(-input.x, input.y));

        return new RunCommand(
                () -> mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                rotated.x,
                                rotated.y
                        ),
                        -rightX.getAsDouble()
                )), this
        );
    }

}


