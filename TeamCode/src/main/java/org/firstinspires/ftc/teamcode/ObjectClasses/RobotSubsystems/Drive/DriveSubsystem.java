package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

import java.util.function.DoubleSupplier;

@Config
public class DriveSubsystem extends SubsystemBase {

    public static class DriveParameters {
        public double DRIVE_SPEED_FACTOR=.8;
        public double STRAFE_SPEED_FACTOR=.8;
        public double TURN_SPEED_FACTOR=.4;
        public double APRIL_TAG_CANCEL_THRESHOLD = -.1;
        public double safetyDriveSpeedFactor = .7;
    }

    public static DriveParameters driveParameters= new DriveParameters();

    public final MecanumDriveMona mecanumDrive;
    public VisionSubsystem visionSubsystem;
    public boolean drivingToAprilTag;
    public boolean fieldOrientedControl;

    public double drive;
    public double strafe;
    public double turn;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;

    public DriveSubsystem(HardwareMap hardwareMap) {
        mecanumDrive = new MecanumDriveMona();
    }

    public void init()
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        drivingToAprilTag=false;
        fieldOrientedControl=false;
        mecanumDrive.init();
    }

    public void setDriveStrafeTurnValues(double leftY, double leftX, double rightX ){

        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive(leftY, leftX, rightX) || drivingToAprilTag) {

            //apply speed factors
            leftYAdjusted = leftY * driveParameters.DRIVE_SPEED_FACTOR;
            leftXAdjusted = leftX * driveParameters.STRAFE_SPEED_FACTOR;
            rightXAdjusted = rightX * driveParameters.TURN_SPEED_FACTOR;

            //adjust stick values if field oriented
            if (fieldOrientedControl){
                fieldOrientedControl(leftYAdjusted, leftXAdjusted);
            }

            // Cancel AprilTag driving if the driver is moving away from the backdrop
            // I'm not sure if this works for field oriented control
            if (leftYAdjusted < driveParameters.APRIL_TAG_CANCEL_THRESHOLD) drivingToAprilTag = false;

            //Align to the Backdrop AprilTags - CASE RED
            if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.RED &&
                    visionSubsystem.redBackdropAprilTagFound &&
                    (leftYAdjusted > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropRed();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (leftYAdjusted > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropBlue();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            } else drivingToAprilTag = false;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            leftYAdjusted = 0;
            leftXAdjusted = 0;
            rightXAdjusted = 0;
        }
        drive = leftYAdjusted;
        strafe = leftXAdjusted;
        turn = rightXAdjusted;
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        mecanumDrive.leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        mecanumDrive.leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        mecanumDrive.rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        mecanumDrive.rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public Command  driveRobotCentric(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        return new RunCommand(
                () -> Robot.getInstance().getDriveSubsystem().setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                            leftY.getAsDouble(),
                            -leftX.getAsDouble()
                    ),
                    -rightX.getAsDouble()
            )), this
        );
    }


    public void fieldOrientedControl (double leftY, double leftX){

        double y = leftY;
        double x = leftX;

        double botHeading = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawRadians;

        // Rotate the movement direction counter to the bot's rotation
        leftXAdjusted = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        leftYAdjusted = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        leftYAdjusted = Math.min( leftYAdjusted * 1.1, 1);  // Counteract imperfect strafing
    }



//    This does not work.
//    public Command driveFieldCentric(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
//        Vector2d input = new Vector2d(
//                leftY.getAsDouble(),
//                -leftX.getAsDouble()
//        );
//
//        Vector2d rotated = mecanumDrive.pose.heading.inverse().times(new Vector2d(-input.x, input.y));
//
//        return new RunCommand(
//                () -> Robot.getInstance().getDriveSubsystem().setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                rotated.x,
//                                rotated.y
//                        ),
//                        -rightX.getAsDouble()
//                )), this
//        );
//    }

}


