package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

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

    public double drive;
    public double strafe;
    public double turn;

    public DriveSubsystem(HardwareMap hardwareMap) {
        mecanumDrive = new MecanumDriveMona();
    }

    public void init()
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        drivingToAprilTag=false;
        mecanumDrive.init();
    }


    public void setDriveStrafeTurnValues(double controllerDrive, double controllerStrafe, double controllerTurn ){

        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive() || drivingToAprilTag) {

            //Increase strafe dead zone to 30%
            if (Math.abs(controllerStrafe) < .3) {controllerStrafe=0;}

            //apply speed factors
            controllerDrive = controllerDrive * driveParameters.DRIVE_SPEED_FACTOR;
            controllerStrafe = controllerStrafe * driveParameters.STRAFE_SPEED_FACTOR;
            controllerTurn = controllerTurn * driveParameters.TURN_SPEED_FACTOR;

            // Cancel AprilTag driving if the driver is moving away from the backdrop
            if (controllerDrive < driveParameters.APRIL_TAG_CANCEL_THRESHOLD) drivingToAprilTag = false;

            //Align to the Backdrop AprilTags - CASE RED
            if (visionSubsystem.getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    visionSubsystem.redBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropRed();
                controllerDrive = mecanumDrive.aprilTagDrive;
                controllerStrafe = mecanumDrive.aprilTagStrafe;
                controllerTurn = mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (visionSubsystem.getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropBlue();
                controllerDrive = mecanumDrive.aprilTagDrive;
                controllerStrafe = mecanumDrive.aprilTagStrafe;
                controllerTurn = mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            } else drivingToAprilTag = false;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = 0;
        }
        drive = controllerDrive;
        strafe = controllerStrafe;
        turn = controllerTurn;
    }














//    public Command driveRobotCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
//        return new RunCommand(
//                () -> mecanumDrive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                            -leftY.getAsDouble(),
//                            -leftX.getAsDouble()
//                    ),
//                    -rightX.getAsDouble()
//            )), this
//        );
//    }
//
//    public Command driveFieldCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
//
//        Vector2d input = new Vector2d(
//                -leftY.getAsDouble(),
//                -leftX.getAsDouble()
//        );
//
//        Vector2d rotated = mecanumDrive.pose.heading.inverse().times(new Vector2d(-input.x, input.y));
//
//        return new RunCommand(
//                () -> mecanumDrive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                rotated.x,
//                                rotated.y
//                        ),
//                        -rightX.getAsDouble()
//                )), this
//        );
//    }

}


