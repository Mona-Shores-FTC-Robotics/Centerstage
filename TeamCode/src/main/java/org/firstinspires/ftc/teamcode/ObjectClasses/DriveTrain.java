package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

public class DriveTrain {
    // DriveTrain tuning constants
    private final double STICK_DEAD_ZONE = .1;

    private double DEFAULT_P = 9.5; // default = 10
    private double DEFAULT_D = 3; // default = 0
    private double DEFAULT_I = 0; // default = 3
    private double DEFAULT_F = 12; // default = 0

    private double P = DEFAULT_P; // default = 10
    private double D = DEFAULT_D; // default = 0
    private double I = DEFAULT_I; // default = 3
    private double F = DEFAULT_F; // default = 0

    // DriveTrain physical constants
    private final double MAX_MOTOR_SPEED_RPS = 312.0 / 60.0;
    public final double TICKS_PER_REV = 537.7;
    private final double DRIVE_GEAR_REDUCTION = 1.0;
    private final double WHEEL_DIAMETER_INCHES = 3.93701;
    private final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    /* Public OpMode objects and variables. */
    public DcMotorEx driveMotor[] = new DcMotorEx[4]; // {leftfront, rightfront, leftback, rightback}
    public final String driveMotorNames [] = {"LFDrive", "RFDrive", "LBDrive", "RBDrive"};
    public double driveMotorTargetSpeed[] = {0.0,0.0,0.0,0.0};
    public double driveMotorPower[] = {0.0,0.0,0.0,0.0};

    public double drive = 0.0;
    public double strafe = 0.0;
    public double turn = 0.0;

    public double aDrive = 0.0;
    public double aStrafe = 0.0;
    public double aTurn = 0.0;

    private final double DRIVE_SPEED_FACTOR = 1;
    private final double STRAFE_SPEED_FACTOR = 1;
    private final double TURN_SPEED_FACTOR = 1;

    private double safetyDriveSpeedFactor = 1.0;
    private double safetyStrafeSpeedFactor = 1.0;
    private double safetyTurnSpeedFactor = 1.0;

    private boolean manualDriveControlFlag = true;
    private boolean fieldOrientedControlFlag = false;
    private boolean backdropSafetyZoneFlag = false;

    private LinearOpMode activeOpMode;
    private Gamepad driverGamepad;

    private double driveInput;
    private double strafeInput;
    private double turnInput;

    /* Constructor */
    public DriveTrain() {
    }

    /* METHOD: Initialize Hardware interfaces */
    public void init() {
        // Save reference to Hardware map
        activeOpMode = Robot.getInstance().getActiveOpMode();

        //set the driverGamepad so we can use the shortname throughout this class
        driverGamepad = GamepadHandling.getCurrentDriverGamepad();

        // Define and Initialize Motors
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rvrs = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction driveMotorDirections[] = {rvrs, fwd, rvrs, fwd};

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = Robot.getInstance().getHardwareMap().get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //driveMotor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            driveMotor[i].setDirection(driveMotorDirections[i]);
        }
    }

    public void drive(){

        P=DEFAULT_P;
        D=DEFAULT_D;
        I=DEFAULT_I;
        F=DEFAULT_F;

        driverGamepad = GamepadHandling.getCurrentDriverGamepad();
        driveInput = -driverGamepad.left_stick_y;
        strafeInput = driverGamepad.left_stick_x;
        turnInput = driverGamepad.right_stick_x;

        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.gamepadIsActive(driverGamepad))
        {
            setManualDriveControlFlag(true);

            if (fieldOrientedControlFlag==true)
            {
                //TODO figure out how to add the safety code to field oriented control driving
                fieldOrientedControl();
            } else {
                drive = driveInput;
                strafe = strafeInput;
                turn = turnInput;

                //Check if we are near the backdrop and scale our movement down if we are trying to move toward the backdrop
                CheckBackdropSafetyZone();
            }

            //call the drive function with the drive/turn/strafe values set based on the driver controls
            mecanumDriveSpeedControl();

        } else if (!getManualDriveControlFlag()) {
            //call the drive function with the drive/turn/strafe values that are already set by vision (or some other system)
            moveRobot(aDrive, aStrafe, aTurn);
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            drive = 0;
            strafe = 0;
            turn = 0;
            mecanumDriveSpeedControl();
        }
    }

    private void CheckBackdropSafetyZone() {
        //this flag is set while looking for AprilTags
        if (backdropSafetyZoneFlag)
        {
            // Only modify if the driver was trying to go forward
            // The size of the adjustments are based on how far away we are from the backdrop AprilTags
            if (driveInput>0) {
                drive = driveInput * safetyDriveSpeedFactor;
                strafe = strafeInput*safetyStrafeSpeedFactor;
                turn = turnInput*safetyTurnSpeedFactor;
            }
        }
    }

    public void mecanumDriveSpeedControl() {
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        driveMotorTargetSpeed[0] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[1] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        driveMotorTargetSpeed[2] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[3] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        for (int i = 0; i < 4; i++ ){
            driveMotor[i].setVelocityPIDFCoefficients(P,I,D,F);
            driveMotor[i].setVelocity(driveMotorTargetSpeed[i]);
        }
    }

    public void mecanumDrivePowerControl (){
        //TODO Should we be trying to do some sort of ramping?

        // Put Mecanum Drive math and motor commands here.
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        double leftFrontPower = ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        double rightFrontPower = ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        double leftBackPower = ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        double rightBackPower = ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        driveMotorPower[0] = leftFrontPower;
        driveMotorPower[1] = rightFrontPower;
        driveMotorPower[2] = leftBackPower;
        driveMotorPower[3] = rightBackPower;


        for (int i = 0; i < 4; i++ ) {
            driveMotor[i].setPower(driveMotorPower[i]);
        }
    }

    /** fieldOrientedControl takes inputs from the gamepad and the angle of the robot.
     * These inputs are used to calculate the drive, strafe and turn inputs needed for the MecanumDrive method.
     */
    public void fieldOrientedControl (){

        double heading = Robot.getInstance().getGyro().getYawDegrees();

        double magnitude = Math.sqrt(Math.pow(drive, 2) + Math.pow(strafe, 2));
        double driveAngle = Math.copySign(Math.acos(drive/magnitude), Math.asin(-strafe));
        double deltaAngle = heading-driveAngle;

        drive = DRIVE_SPEED_FACTOR * magnitude * Math.cos(deltaAngle);
        strafe = STRAFE_SPEED_FACTOR * magnitude * Math.sin(deltaAngle);
        turn = TURN_SPEED_FACTOR * turn;

    }

    public void setAutoDrive(double autoDrive) { aDrive = autoDrive;}
    public void setAutoStrafe(double autoStrafe) { aStrafe = autoStrafe;}
    public void setAutoTurn(double autoTurn) {  aTurn = autoTurn;  }

    public void setManualDriveControlFlag(boolean flag) {
        manualDriveControlFlag = flag;
    }

    public boolean getManualDriveControlFlag() {
        return manualDriveControlFlag;
    }

    public void setFieldOrientedControlFlag(boolean flag) {
        fieldOrientedControlFlag = flag;
    }

    public boolean getFieldOrientedControlFlag() {
        return fieldOrientedControlFlag;
    }

    public void setSafeyDriveSpeedFactor(double factor) { safetyDriveSpeedFactor = factor;}

    public void setSafetyStrafeSpeedFactor(double factor) { safetyStrafeSpeedFactor = factor; }

    public void setSafetyTurnSpeedFactor(double factor) {
        safetyTurnSpeedFactor = factor;
    }

    public void setBackdropSafetyZone(boolean flag) {
        backdropSafetyZoneFlag = flag;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        driveMotor[0].setPower(leftFrontPower);
        driveMotor[1].setPower(rightFrontPower);
        driveMotor[2].setPower(leftBackPower);
        driveMotor[3].setPower(rightBackPower);
    }

    public void telemetryDriveTrain() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");
        for (int i = 0; i < 4; i++ ){
            double targetSpeed = Math.round(100.0 * driveMotorTargetSpeed[i] / TICKS_PER_REV);
            double actualSpeed = Math.round(100.0 * driveMotor[i].getVelocity() / TICKS_PER_REV);
            double power =  Math.round(100.0 * driveMotorPower[i])/100.0;
            activeOpMode.telemetry.addLine("Motor " + i + " Speed: " + JavaUtil.formatNumber(actualSpeed, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeed, 4, 1)  + " " + "Power: " +  Math.round(100.0 * driveMotor[i].getPower())/100.0);
        }
    }
}
