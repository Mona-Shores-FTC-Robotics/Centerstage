package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class GyroSubsystem extends SubsystemBase {

    private IMU imu;

    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.UP); // direction of USB ports on robot

    public double currentAbsoluteYawDegrees;
    public double currentAbsoluteYawRadians;
    private double currentRelativeYaw;
    private double lastRelativeYaw;


    public GyroSubsystem(final HardwareMap hMap, final String name) {
        imu = hMap.get(IMU.class, name);
    }

    public void init() {
        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();
    }

    public void UpdateGyro() {
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
    }

    public void resetAbsoluteYaw() {
        imu.resetYaw();
    }

    public void resetRelativeYaw(){
        lastRelativeYaw = currentAbsoluteYawDegrees;
        currentRelativeYaw = 0;
    }

    public double getCurrentRelativeYaw()
    {
        double deltaAngle = currentAbsoluteYawDegrees - lastRelativeYaw;

        if (deltaAngle>180){
            deltaAngle-=360;
        } else if(deltaAngle <=-180)
        {
            deltaAngle +=360;
        }
        currentRelativeYaw+= deltaAngle;
        lastRelativeYaw = currentAbsoluteYawDegrees;
        telemetryGyro();
        return currentRelativeYaw;
    }

    public void telemetryGyro() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle (Degrees)" + JavaUtil.formatNumber(currentAbsoluteYawDegrees, 4, 0));
    }

    public IMU getIMU() {
        return imu;
    }
}
