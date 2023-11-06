package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParameters;

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

    public static class ParamsMona {
        public double DRIVE_RAMP = .06; //ken ramp
        public double STRAFE_RAMP = .05;
        public double TURN_RAMP = .05;

        public double RAMP_THRESHOLD = .04; // This is the threshold at which we just clamp to the target drive/strafe/turn value

        //it looks to me like just using a feedforward of 12.5 gets the actual speed to match the target. The PID doesn't seem to really do anything.
        public double P =0; // default = 10
        public double D =0; // default = 0
        public double I =0; // default = 3
        public double F =8; // default = 0
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

    public void periodic(){
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
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

    public void fieldOrientedControl (double leftY, double leftX){
        double y = leftY;
        double x = leftX;
        double botHeading = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawRadians;

        // Rotate the movement direction counter to the bot's rotation
        leftXAdjusted = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        leftYAdjusted = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        leftYAdjusted = Math.min( leftYAdjusted * 1.1, 1);  // Counteract imperfect strafing
    }
}


