package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.DriveTrainConstants;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParameters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
        /** Set Roadrunner motor parameters for faster drive motors **/

        // drive model parameters
        public double inPerTick =0.0317919075144509; //60.5\1903
        public double lateralInPerTick =0.0325115144947169; // 60\1845.5
        public double trackWidthTicks =631.8289216104534;

        // feedforward parameters (in tick units)
        public double kS =0.9574546275336608;
        public double kV =0.004264232249424524;
        public double kA =0.00055;

        // path profile parameters (in inches)
        public double maxWheelVel =25;
        public double minProfileAccel =-30;
        public double maxProfileAccel =30;

        // turn profile parameters (in radians)
        public double maxAngVel =Math.PI; // shared with path
        public double maxAngAccel =Math.PI;

        // path controller gains
        public double axialGain =12;
        public double lateralGain =8;
        public double headingGain =8; // shared with turn

        public double axialVelGain =1.1;
        public double lateralVelGain =1.1;
        public double headingVelGain =1.1; // shared with turn
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
        c = new Canvas();
    }

    public void periodic(){
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("current_drive_ramp", mecanumDrive.current_drive_ramp);
        packet.put("current_strafe_ramp", mecanumDrive.current_strafe_ramp);
        packet.put("current_turn_ramp", mecanumDrive.current_turn_ramp);

        double targetSpeedLF = Math.round(100.0 * mecanumDrive.leftFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedRF = Math.round(100.0 * mecanumDrive.rightFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedLB = Math.round(100.0 * mecanumDrive.leftBackTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedRB = Math.round(100.0 * mecanumDrive.rightBackTargetSpeed / DriveTrainConstants.TICKS_PER_REV);

        double actualSpeedLF = Math.round(100.0 * mecanumDrive.leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedRF = Math.round(100.0 * mecanumDrive.rightFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedLB = Math.round(100.0 * mecanumDrive.leftBack.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedRB = Math.round(100.0 * mecanumDrive.rightBack.getVelocity() / DriveTrainConstants.TICKS_PER_REV);

        packet.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * mecanumDrive.leftFront.getPower()) / 100.0);
        packet.addLine("RF" + " Speed: " + JavaUtil.formatNumber(actualSpeedRF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRF, 4, 1) + " " + "Power: " + Math.round(100.0 * mecanumDrive.rightFront.getPower()) / 100.0);
        packet.addLine("LB" + " Speed: " + JavaUtil.formatNumber(actualSpeedLB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLB, 4, 1) + " " + "Power: " + Math.round(100.0 * mecanumDrive.leftBack.getPower()) / 100.0);
        packet.addLine("RB" + " Speed: " + JavaUtil.formatNumber(actualSpeedRB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRB, 4, 1) + " " + "Power: " + Math.round(100.0 * mecanumDrive.rightBack.getPower()) / 100.0);

        packet.fieldOverlay().getOperations().addAll(c.getOperations());
        packet.put("x", mecanumDrive.pose.position.x);
        packet.put("y", mecanumDrive.pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(mecanumDrive.pose.heading.log()));

        Canvas c = packet.fieldOverlay();
        mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        mecanumDrive.drawRobot(c, mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
                drivingToAprilTag = visionSubsystem.AutoDriveToBackdropRed();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
            }
            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (leftYAdjusted > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                drivingToAprilTag = visionSubsystem.AutoDriveToBackdropBlue();
                leftYAdjusted = mecanumDrive.aprilTagDrive;
                leftXAdjusted = mecanumDrive.aprilTagStrafe;
                rightXAdjusted = mecanumDrive.aprilTagTurn;
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


