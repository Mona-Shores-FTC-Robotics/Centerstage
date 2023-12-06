package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

@Config
public class DriveSubsystem extends SubsystemBase {

    public static class ParamsDriveTrainConstants {
        // DriveTrain physical constants
        public double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        public double TICKS_PER_REV = 384.5;
        public double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;
    }

    public static class DriveParameters {
        public double DRIVE_SPEED_FACTOR=.9;
        public double STRAFE_SPEED_FACTOR=.9;
        public double TURN_SPEED_FACTOR=.8;
        public double APRIL_TAG_CANCEL_THRESHOLD = -.1;
        public double safetyDriveSpeedFactor = .7;
        public double DEAD_ZONE = .2;
        public double APRILTAG_AUTODRIVING_TIMEOUT_THRESHOLD=3;
        public double DRIVE_THRESHOLD_FOR_APRILTAG_DRIVING= .4;
    }

    public static class ParamsMona {
        /** Set Mona motor parameters for faster TeleOp driving**/
        public double DRIVE_RAMP = .2; //ken ramp
        public double STRAFE_RAMP = .22;
        public double TURN_RAMP = .4;

        public double RAMP_THRESHOLD = .04; // This is the threshold at which we just clamp to the target drive/strafe/turn value

        //it looks to me like just using a feedforward of 12.5 gets the actual speed to match the target. The PID doesn't seem to really do anything.
        public double P =0; // default = 10
        public double D =0; // default = 0
        public double I =0; // default = 3
        public double F =8; // default = 0
    }

    public static DriveParameters driveParameters= new DriveParameters();
    public static DriveSubsystem.ParamsMona MotorParameters = new DriveSubsystem.ParamsMona();
    public static ParamsDriveTrainConstants DriveTrainConstants = new ParamsDriveTrainConstants();


    public enum DriveStates {
        MANUAL_DRIVE,
        SLOW_MANUAL_DRIVE,
        STRAFING_X_INCHES,
        BACKUP_FROM_BACKDROP,
        APRIL_TAG_ALIGNMENT_TURNING,
        APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG,
        APRIL_TAG_ALIGNMENT_STRAFING,
        APRIL_TAG_ALIGNMENT_DRIVING;
    }

    public DriveStates currentState = DriveStates.MANUAL_DRIVE;

  // AprilTag Driving Variables
    private boolean overrideAprilTagDriving = false;
    public double drive=0, strafe=0, turn=0;
    public double aprilTagDrive=0, aprilTagStrafe=0, aprilTagTurn=0;
    public double last_drive=0, last_strafe=0, last_turn=0;
    public double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;
    public double leftFrontTargetSpeed=0, rightFrontTargetSpeed=0, leftBackTargetSpeed=0, rightBackTargetSpeed=0;

    public ElapsedTime aprilTagTimeoutTimer = new ElapsedTime();

    public MecanumDrive mecanumDrive;

    public VisionSubsystem visionSubsystem;
    public boolean aprilTagAutoDriving;
    public boolean fieldOrientedControl;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;

    public Canvas c;

    public DriveSubsystem(HardwareMap hardwareMap) {

    }

    private GamepadHandling gamepadHandling;

    public void init(GamepadHandling gpadHandling)
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        gamepadHandling = gpadHandling;
        aprilTagAutoDriving =false;
        fieldOrientedControl=false;
        currentState = DriveStates.MANUAL_DRIVE;
        mecanumDrive = new MecanumDrive(Robot.getInstance().getActiveOpMode().hardwareMap, new Pose2d(0,0,0));
        overrideAprilTagDriving = false;
        drive=0; strafe=0; turn=0;
        aprilTagDrive=0; aprilTagStrafe=0; aprilTagTurn=0;
        last_drive=0; last_strafe=0; last_turn=0;
        current_drive_ramp = 0; current_strafe_ramp=0; current_turn_ramp=0;
        leftFrontTargetSpeed=0; rightFrontTargetSpeed=0; leftBackTargetSpeed=0; rightBackTargetSpeed=0;
    }

    public void periodic(){
        //If the vision subsystem has a pose ready for us (because we are at the backdrop)
        // AND the driver set the resetHeading flag (this is the SHARE button right now)
        //      then:
        //          1) reset the gyro (to 0)
        //          2) change the relativeYaw to 0 (offset 0 from gyro)
        //          3) update the robot pose
        //              i) set the X/Y coordinates based on the april tag
        //              ii)set the heading to 0

        if (visionSubsystem.resetPoseReady){
            if (Robot.getInstance().getVisionSubsystem().resetHeading) {
                Robot.getInstance().getVisionSubsystem().resetHeading=false;
                Robot.getInstance().getGyroSubsystem().setRelativeYawTo0();
                visionSubsystem.resetPoseReady = false;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose =
                        new Pose2d (visionSubsystem.resetPose.position.x,
                                visionSubsystem.resetPose.position.y,
                                0);
                Robot.getInstance().getDriveSubsystem().mecanumDrive.poseHistory.clear();
            }

            //Else the vision subsystem has a pose ready for us (because we are at the backdrop), but we don't want to reset the gyro
            //      THEN:
            //          Update the robot pose with the reset Pose
            //              i) sets the X/Y coordinates based on the april tag
            //              ii)sets the heading to the currentRelativeYaw

            else {
                visionSubsystem.resetPoseReady = false;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = visionSubsystem.resetPose;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.poseHistory.clear();
            }
        }

        DashboardTelemetryDriveTrain();
    }

    private void DashboardTelemetryDriveTrain() {
        //****************** DriveSubsystem TELEMETRY PACKET *************************//

        c = MatchConfig.telemetryPacket.fieldOverlay();

        //Lets just look at the Left Front wheel speed to get an idea of speed of the robot

        double powerLF = Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getPower();

        MatchConfig.telemetryPacket.put("Drive Subsystem State: ", currentState);
        MatchConfig.telemetryPacket.put("LF Power", powerLF);

        MatchConfig.telemetryPacket.put("x", mecanumDrive.pose.position.x);
        MatchConfig.telemetryPacket.put("y", mecanumDrive.pose.position.y);
        MatchConfig.telemetryPacket.put("heading (deg)", Math.toDegrees(mecanumDrive.pose.heading.log()));

        MatchConfig.telemetryPacket.put("April Tag Override Status: ", overrideAprilTagDriving);

        MatchConfig.telemetryPacket.fieldOverlay().getOperations().addAll(c.getOperations());
        mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        mecanumDrive.drawRobot(c, mecanumDrive.pose);
    }


    public Boolean driverGamepadIsActive(double leftY, double leftX, double rightX) {
        if     (Math.abs(leftY) > driveParameters.DEAD_ZONE ||
                Math.abs(leftX) > driveParameters.DEAD_ZONE ||
                Math.abs(rightX) > driveParameters.DEAD_ZONE ){
            return true;
        } else return false;
    }

    public void setDriveStrafeTurnValues(double leftY, double leftX, double rightX ){

        boolean gamepadActive = driverGamepadIsActive(leftY, leftX, rightX);
        //Check if driver controls are active so we can cancel automated driving if they are
        if (gamepadActive || aprilTagAutoDriving) {

            //apply speed factors
            leftYAdjusted = leftY * driveParameters.DRIVE_SPEED_FACTOR;
            leftXAdjusted = leftX * driveParameters.STRAFE_SPEED_FACTOR;
            rightXAdjusted = rightX * driveParameters.TURN_SPEED_FACTOR;

            //adjust stick values if field oriented
            if (fieldOrientedControl){
                fieldOrientedControl(leftYAdjusted, leftXAdjusted);
            }

            // Cancel AprilTag driving if the driver is moving away from the backdrop
            if (leftYAdjusted < driveParameters.APRIL_TAG_CANCEL_THRESHOLD ||
                Math.abs(leftXAdjusted) > .3){
                aprilTagAutoDriving = false;
            }


            if (    ((MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE && visionSubsystem.blueBackdropAprilTagFoundRecently) ||
                    (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED && visionSubsystem.redBackdropAprilTagFoundRecently)) &&
                    (leftYAdjusted > driveParameters.DRIVE_THRESHOLD_FOR_APRILTAG_DRIVING || aprilTagAutoDriving) &&
                    !getOverrideAprilTagDriving()) {
                //start apriltag timeout timer
                if (!aprilTagAutoDriving) {
                    aprilTagTimeoutTimer.reset();
                }
                aprilTagAutoDriving = visionSubsystem.AutoDriveToBackdrop();
                leftYAdjusted = aprilTagDrive;
                leftXAdjusted = aprilTagStrafe;
                rightXAdjusted = aprilTagTurn;
                MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(aprilTagDrive, 6, 6));
                MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(aprilTagStrafe, 6, 6));
                MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(aprilTagTurn, 6, 6));
                MatchConfig.telemetryPacket.put("AprilTag Range Error", visionSubsystem.rangeError);
                MatchConfig.telemetryPacket.put("AprilTag Yaw Error", visionSubsystem.yawError);
                MatchConfig.telemetryPacket.put("AprilTag xError Error", visionSubsystem.xError);
                MatchConfig.telemetryPacket.put("AprilTag Bearing Error", visionSubsystem.bearingError);

                //Check if we timed out
                if (aprilTagTimeoutTimer.seconds() > driveParameters.APRILTAG_AUTODRIVING_TIMEOUT_THRESHOLD) {
                    aprilTagAutoDriving=false;

                    drive=0; strafe=0; turn=0;
                    current_drive_ramp = 0; current_strafe_ramp=0; current_turn_ramp=0;
                    aprilTagDrive=0; aprilTagStrafe=0; aprilTagTurn=0;

                }

            } else aprilTagAutoDriving = false;

//            //Align to the Backdrop AprilTags - CASE RED
//            if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED &&
//                    visionSubsystem.redBackdropAprilTagFoundRecently &&
//                    (leftYAdjusted > driveParameters.DRIVE_THRESHOLD_FOR_APRILTAG_DRIVING || aprilTagAutoDriving) &&
//                    !getOverrideAprilTagDriving()) {
//                //start apriltag timeout timer
//                if (!aprilTagAutoDriving) {
//                    aprilTagTimeoutTimer.reset();
//                }
//                aprilTagAutoDriving=visionSubsystem.AutoDriveToBackdropRed();
//                leftYAdjusted = mecanumDrive.aprilTagDrive;
//                leftXAdjusted = mecanumDrive.aprilTagStrafe;
//                rightXAdjusted = mecanumDrive.aprilTagTurn;
//
//                MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(mecanumDrive.aprilTagDrive, 6, 6));
//                MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(mecanumDrive.aprilTagStrafe, 6, 6));
//                MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(mecanumDrive.aprilTagTurn, 6, 6));
//                MatchConfig.telemetryPacket.put("AprilTag Range Error", visionSubsystem.rangeError);
//                MatchConfig.telemetryPacket.put("AprilTag Yaw Error", visionSubsystem.yawError);
//                MatchConfig.telemetryPacket.put("AprilTag xError Error", visionSubsystem.xError);
//
//                //Check if we timed out
//                if (aprilTagTimeoutTimer.seconds() > driveParameters.APRILTAG_AUTODRIVING_TIMEOUT_THRESHOLD) {
//                    aprilTagAutoDriving=false;
//                }
//
//            }
//            //Aligning to the Backdrop AprilTags - CASE BLUE
//            else if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE &&
//                    visionSubsystem.blueBackdropAprilTagFoundRecently &&
//                    (leftYAdjusted > driveParameters.DRIVE_THRESHOLD_FOR_APRILTAG_DRIVING || aprilTagAutoDriving) &&
//                    !getOverrideAprilTagDriving()) {
//                //start apriltag timeout timer
//                if (!aprilTagAutoDriving) {
//                    aprilTagTimeoutTimer.reset();
//                }
//                aprilTagAutoDriving = visionSubsystem.AutoDriveToBackdropBlue();
//                leftYAdjusted = mecanumDrive.aprilTagDrive;
//                leftXAdjusted = mecanumDrive.aprilTagStrafe;
//                rightXAdjusted = mecanumDrive.aprilTagTurn;
//                MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(mecanumDrive.aprilTagDrive, 6, 6));
//                MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(mecanumDrive.aprilTagStrafe, 6, 6));
//                MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(mecanumDrive.aprilTagTurn, 6, 6));
//                MatchConfig.telemetryPacket.put("AprilTag Range Error", visionSubsystem.rangeError);
//                MatchConfig.telemetryPacket.put("AprilTag Yaw Error", visionSubsystem.yawError);
//                MatchConfig.telemetryPacket.put("AprilTag xError Error", visionSubsystem.xError);
//
//
//                //Check if we timed out
//                if (aprilTagTimeoutTimer.seconds() > driveParameters.APRILTAG_AUTODRIVING_TIMEOUT_THRESHOLD) {
//                    aprilTagAutoDriving=false;
//                }
//
//            } else aprilTagAutoDriving = false;
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
        double botHeading;

        //This should make it so field centric driving works for both alliance colors
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
            botHeading = Math.toRadians(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees - 90);
        } else {
            botHeading = Math.toRadians(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees + 90);
        }

        // Rotate the movement direction counter to the bot's rotation
        leftXAdjusted = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        leftYAdjusted = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        leftYAdjusted = Math.min( leftYAdjusted * 1.1, 1);  // Counteract imperfect strafing
    }

    public boolean getOverrideAprilTagDriving() {
        return overrideAprilTagDriving;
    }
    public void setOverrideAprilTagDriving(boolean b) {
        overrideAprilTagDriving=b;
    }

    // Method to get the current average encoder count
    public double getCurrentEncoderCount() {
        double fl = mecanumDrive.leftFront.getCurrentPosition();
        double fr = mecanumDrive.rightFront.getCurrentPosition();
        double bl = mecanumDrive.leftBack.getCurrentPosition();
        double br = mecanumDrive.rightBack.getCurrentPosition();

        return (fl + fr + bl + br) / 4.0;
    }

    // Method to reset encoders
    public void resetEncoders() {
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // After resetting, you might need to set them back to the desired run mode
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {

        //I believe this is the best spot to put this for tracking our pose
        //We had this method call in the periodic() of the Drivesystem, but that means it can be called twice a loop if we run any RR actions (e.g.follow a trajectory)
        //putting it here, should avoid that double call

        if (drive==0 && strafe ==0 && turn==0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            mecanumDrive.leftFront.setVelocity(0);
            mecanumDrive.leftBack.setVelocity(0);
            mecanumDrive.rightFront.setVelocity(0);
            mecanumDrive.rightBack.setVelocity(0);

            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);

            current_drive_ramp=0;
            current_strafe_ramp=0;
            current_turn_ramp=0;

        } else
        {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            //safetydrivespeedfactor is set when we lookforapriltags based on the closest backdrop apriltag we see (for the opposite alliance color)
            if (Robot.getInstance().getVisionSubsystem().blueBackdropAprilTagFoundRecently &&
                    MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED &&
                    drive > .2) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }
            //If we see red tags and we are blue and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            else if (Robot.getInstance().getVisionSubsystem().redBackdropAprilTagFoundRecently &&
                    MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    drive > .2) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }

            current_drive_ramp = Ramp(drive, current_drive_ramp, MotorParameters.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, MotorParameters.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, MotorParameters.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            leftFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            mecanumDrive.leftFront.setVelocity(leftFrontTargetSpeed);
            mecanumDrive.rightFront.setVelocity(rightFrontTargetSpeed);
            mecanumDrive.leftBack.setVelocity(leftBackTargetSpeed);
            mecanumDrive.rightBack.setVelocity(rightBackTargetSpeed);

            last_drive=drive;
            last_strafe=strafe;
            last_turn=turn;
        }
    }

    private double Ramp(double target, double currentValue, double ramp_amount) {
        if (Math.abs(currentValue) + MotorParameters.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }


    public void mecanumDrivePowerControl (){
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        double leftFrontPower = ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        double rightFrontPower = ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        double leftBackPower = ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        double rightBackPower = ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        mecanumDrive.leftFront.setPower(leftFrontPower);
        mecanumDrive.rightFront.setPower(rightFrontPower);
        mecanumDrive.leftBack.setPower(leftBackPower);
        mecanumDrive.rightBack.setPower(rightBackPower);
    }

}
