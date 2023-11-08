package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class GyroSubsystem extends SubsystemBase {

    private IMU imu;

    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.UP); // direction of USB ports on robot

    public double currentAbsoluteYawDegrees;
    public double currentAbsoluteYawRadians;

    public double currentRelativeYawDegrees;
    public double currentRelativeYawRadians;

    private double lastRelativeYawDegrees;

    public GyroSubsystem(final HardwareMap hMap, final String name) {
        imu = hMap.get(IMU.class, name);
    }

    public void init() {
        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();
    }

    public void periodic() {
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
    }

    public void resetAbsoluteYaw() {
        imu.resetYaw();
        resetRelativeYaw();
    }

    public void resetRelativeYaw() {
        lastRelativeYawDegrees = currentAbsoluteYawDegrees;
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
            currentRelativeYawDegrees = 90;
            currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);
        } else {
            currentRelativeYawDegrees = -90;
            currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);
        }
    }

    public void setRelativeYawTo0(){
        lastRelativeYawDegrees = currentAbsoluteYawDegrees;
        currentRelativeYawDegrees = 0;
        currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);
    }

    public double getCurrentRelativeYaw(){
        double deltaAngle = currentAbsoluteYawDegrees - lastRelativeYawDegrees;

        if (deltaAngle>180){
            deltaAngle-=360;
        } else if(deltaAngle <=-180)
        {
            deltaAngle +=360;
        }
        currentRelativeYawDegrees+= deltaAngle;
        currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);

        lastRelativeYawDegrees = currentAbsoluteYawDegrees;
        telemetryGyro();
        return currentRelativeYawDegrees;
    }

    public void telemetryGyro() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Abs (Degrees)" + JavaUtil.formatNumber(currentAbsoluteYawDegrees, 4, 0));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Rel (Degrees)" + JavaUtil.formatNumber(currentRelativeYawDegrees, 4, 0));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Robot Pose (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log(), 4, 0));
    }

}
