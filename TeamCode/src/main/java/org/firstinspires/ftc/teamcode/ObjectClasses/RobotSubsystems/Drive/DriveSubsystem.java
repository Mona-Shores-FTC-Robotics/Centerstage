package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.DriveTrainConstants;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParameters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

@Config
public class DriveSubsystem extends SubsystemBase {

    public static class DriveParameters {
        /** Set these drive parameters for faster TeleOp driving**/
        public double DRIVE_SPEED_FACTOR=.9;
        public double STRAFE_SPEED_FACTOR=.9;
        public double TURN_SPEED_FACTOR=.8;
        public double APRIL_TAG_CANCEL_THRESHOLD = -.1;
        public double safetyDriveSpeedFactor = .7;
        public double DEAD_ZONE = .1;
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

    public static class ParamsRRMona {
        /** Set Roadrunner motor parameters for faster roadrunner trajectories **/

        // drive model parameters
        public double inPerTick =0.0317919075144509; //60.5\1903
        public double lateralInPerTick =0.029; // 60\1845.5
        public double trackWidthTicks =631.8289216104534;

        // feedforward parameters (in tick units)
        public double kS =0.9574546275336608;
        public double kV =0.004264232249424524;
        public double kA =0.00055;

        // path profile parameters (in inches)
        public double maxWheelVel =40;
        public double minProfileAccel =-45;
        public double maxProfileAccel =45;

        // turn profile parameters (in radians)
        public double maxAngVel =Math.PI; // shared with path
        public double maxAngAccel =Math.PI;

        //These are being used in the run part of the trajectory and turn action so they should be live updating.
        // path controller gains
        public double axialGain =12;
        public double lateralGain =10;
        public double headingGain =4; // shared with turn

        public double axialVelGain =1.1;
        public double lateralVelGain =1.1;
        public double headingVelGain =1.1; // shared with turn
    }

    public static DriveParameters driveParameters= new DriveParameters();

    public MecanumDriveMona mecanumDrive;

    private boolean overrideAprilTagDriving = false;

    public VisionSubsystem visionSubsystem;
    public boolean drivingToAprilTag;
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

    public void init()
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        drivingToAprilTag=false;
        fieldOrientedControl=false;
        mecanumDrive.init();
    }

    public void periodic(){

        //If the vision subsystem has a pose ready for us (because we are at the backdrop)
        // AND the driver set the resetHeading flag (this is the BACK button right now)
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

        //Prepare and send a telemetry packet with drive information
//        DashboardTelemetryDriveTrain();

        //todo can we make this more robust so all drive parameters are being live updated in the dashboard?
        mecanumDrive.SetRoadRunnerParameters();

        //update the PIDFCoefficients every loop so that changes we make in the dashboard take effect
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
    }

    private void DashboardTelemetryDriveTrain() {
        //****************** DriveSubsystem TELEMETRY PACKET *************************//
        TelemetryPacket packet = new TelemetryPacket();
        c = packet.fieldOverlay();

        //Lets just look at the Left Front wheel speed to get an idea of speed of the robot
        double targetSpeedLF = Math.round(100.0 * mecanumDrive.leftFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedLF = Math.round(100.0 * mecanumDrive.leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double powerLF = Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getPower();

        packet.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * powerLF) / 100.0);
        packet.put("LF Speed", actualSpeedLF);
        packet.put("LF Target", targetSpeedLF);
        packet.put("LF Power", powerLF);

        packet.put("x", mecanumDrive.pose.position.x);
        packet.put("y", mecanumDrive.pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(mecanumDrive.pose.heading.log()));

        packet.fieldOverlay().getOperations().addAll(c.getOperations());
        mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        mecanumDrive.drawRobot(c, mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

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
        if (gamepadActive || drivingToAprilTag) {

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
                    !getOverrideAprilTagDriving()) {
                drivingToAprilTag = visionSubsystem.AutoDriveToBackdropRed();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
                TelemetryPacket telemetryPacket = new TelemetryPacket();
                telemetryPacket.put("April Tag Drive", mecanumDrive.aprilTagDrive);
                telemetryPacket.put("April Tag Strafe", mecanumDrive.aprilTagStrafe);
                telemetryPacket.put("April Tag Turn", mecanumDrive.aprilTagTurn);
                FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

            }
            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (leftYAdjusted > .1 || drivingToAprilTag) &&
                    !getOverrideAprilTagDriving()) {
                drivingToAprilTag = visionSubsystem.AutoDriveToBackdropBlue();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
                TelemetryPacket telemetryPacket = new TelemetryPacket();
                telemetryPacket.put("April Tag Drive", mecanumDrive.aprilTagDrive);
                telemetryPacket.put("April Tag Strafe", mecanumDrive.aprilTagStrafe);
                telemetryPacket.put("April Tag Turn", mecanumDrive.aprilTagTurn);
                FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
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
        double botHeading = Math.toRadians(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees-90);

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

}


