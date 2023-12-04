package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.DriveTrainConstants;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParameters;

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
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

@Config
public class DriveSubsystem extends SubsystemBase {

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

    public ElapsedTime aprilTagTimeoutTimer = new ElapsedTime();

    public MecanumDriveMona mecanumDrive;

    private boolean overrideAprilTagDriving = false;

    public VisionSubsystem visionSubsystem;
    public boolean aprilTagAutoDriving;
    public boolean fieldOrientedControl;

    public double drive;
    public double strafe;
    public double turn;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;

    public Canvas c;

    public DriveSubsystem(HardwareMap hardwareMap) {
        mecanumDrive = new MecanumDriveMona();
    }

    private GamepadHandling gamepadHandling;

    public void init(GamepadHandling gpadHandling)
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        gamepadHandling = gpadHandling;
        aprilTagAutoDriving =false;
        fieldOrientedControl=false;
        mecanumDrive.init();
        currentState = DriveStates.MANUAL_DRIVE;
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
        mecanumDrive.SetRoadRunnerParameters();

        //update the PIDFCoefficients every loop so that changes we make in the dashboard take effect
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
    }

    private void DashboardTelemetryDriveTrain() {
        //****************** DriveSubsystem TELEMETRY PACKET *************************//

        c = MatchConfig.telemetryPacket.fieldOverlay();

        //Lets just look at the Left Front wheel speed to get an idea of speed of the robot
        double targetSpeedLF = Math.round(100.0 * mecanumDrive.leftFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedLF = Math.round(100.0 * mecanumDrive.leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double powerLF = Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getPower();

        MatchConfig.telemetryPacket.put("Drive Subsystem State: ", currentState);
        MatchConfig.telemetryPacket.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * powerLF) / 100.0);
        MatchConfig.telemetryPacket.put("LF Speed", actualSpeedLF);
        MatchConfig.telemetryPacket.put("LF Target", targetSpeedLF);
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

    public void DriverStationTelemetry() {
        double targetSpeedLF = Math.round(100.0 * Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBackTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedLF = Math.round(100.0 * Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);

        Robot.getInstance().getActiveOpMode().telemetry.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) +
                "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getPower()) / 100.0);
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
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
                MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(mecanumDrive.aprilTagDrive, 6, 6));
                MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(mecanumDrive.aprilTagStrafe, 6, 6));
                MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(mecanumDrive.aprilTagTurn, 6, 6));
                MatchConfig.telemetryPacket.put("AprilTag Range Error", visionSubsystem.rangeError);
                MatchConfig.telemetryPacket.put("AprilTag Yaw Error", visionSubsystem.yawError);
                MatchConfig.telemetryPacket.put("AprilTag xError Error", visionSubsystem.xError);
                MatchConfig.telemetryPacket.put("AprilTag Bearing Error", visionSubsystem.bearingError);

                //Check if we timed out
                if (aprilTagTimeoutTimer.seconds() > driveParameters.APRILTAG_AUTODRIVING_TIMEOUT_THRESHOLD) {
                    aprilTagAutoDriving=false;

                    mecanumDrive.drive=0; mecanumDrive.strafe=0; mecanumDrive.turn=0;
                    mecanumDrive.current_drive_ramp = 0; mecanumDrive.current_strafe_ramp=0; mecanumDrive.current_turn_ramp=0;
                    mecanumDrive.aprilTagDrive=0; mecanumDrive.aprilTagStrafe=0; mecanumDrive.aprilTagTurn=0;

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
}
