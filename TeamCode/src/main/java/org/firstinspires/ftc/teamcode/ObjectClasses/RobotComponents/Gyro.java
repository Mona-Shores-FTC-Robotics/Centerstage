package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.LinkedList;

public class Gyro {

    private LinearOpMode activeOpMode;
    private HardwareMap hwMap;
    private IMU imu;


    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT); // direction of USB ports on robot

    public double currentAbsoluteYawDegrees;
    public double currentAbsoluteYawRadians;
    private double currentRelativeYaw;
    private double lastRelativeYaw;


    public Gyro() {

    }

    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        hwMap = Robot.getInstance().getHardwareMap();
        imu = hwMap.get(IMU.class, "imu");
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
