package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

public class DriveTrain {
    // DriveTrain tuning constants
    private final double STICK_DEAD_ZONE = .1;

    private double P = 14; // default = 10
    private double D = 0; // default = 0
    private double I = 0; // default = 3
    private double F = 0; // default = 0

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

    private double autoDriveSpeedFactor = 1.0;
    private double autoStrafeSpeedFactor = 1.0;
    private double autoTurnSpeedFactor = 1.0;

    private boolean manualDriveControlFlag = true;
    private boolean fieldOrientedControlFlag = false;
    private boolean backdropSafetyZoneFlag = false;

    private LinearOpMode activeOpMode;
    private Gamepad driverGamepad;

    /* Constructor */
    public DriveTrain() {

    }

    /* METHOD: Initialize Hardware interfaces */
    public void init() {
        // Save reference to Hardware map
        activeOpMode = Robot.getInstance().getActiveOpMode();

        //set the driverGamepad so we can use the shortname throughout this class
        driverGamepad = Robot.getInstance().getActiveOpMode().gamepad1;

        // Manual Control is allowed at init
        setManualDriveControlFlag(true);


        //MecanumDrive drivetrain = new MecanumDrive(Robot.getInstance().getHardwareMap(), new Pose2d(0,0,0));

//        // Define and Initialize Motors
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rvrs = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction driveMotorDirections[] = {rvrs, fwd, rvrs, fwd};

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = Robot.getInstance().getHardwareMap().get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setDirection(driveMotorDirections[i]);
        }
    }

    public void drive(){

        //can we just add the speed factor here instead of doing it in field control only?
        double driveInput = -driverGamepad.left_stick_y;
        double strafeInput = driverGamepad.left_stick_x;
        double turnInput = driverGamepad.right_stick_x;

        //Check if the driver sticks are being moved at all so we can cancel any automated driving
        if (    Math.abs(driveInput) > STICK_DEAD_ZONE ||
                Math.abs(strafeInput) > STICK_DEAD_ZONE ||
                Math.abs(turnInput) > STICK_DEAD_ZONE)
        {
            setManualDriveControlFlag(true);

            if (fieldOrientedControlFlag==true)
            {
                // set the drive/turn/strafe without speed adjustment (will be set after calcs for FOC)
                drive = driveInput;
                turn = turnInput;
                strafe = strafeInput;

                //change the drive/strafe/turn values to FOC style
                fieldOrientedControl();
            } else {
                if (backdropSafetyZoneFlag)
                {
                    // for now we are only changing the autoDriveSpeedFactor based on range to apriltag of backdrop
                    if (driveInput>0) {
                        drive = driveInput * autoDriveSpeedFactor;
                    } else if (driveInput <0)
                    {
                        drive = driveInput;
                    }


                    strafe = strafeInput*autoStrafeSpeedFactor;
                    turn = turnInput*autoTurnSpeedFactor;

                } else {
                    drive = driveInput;
                    strafe = strafeInput;
                    turn = turnInput;
                }
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

    public void mecanumDriveSpeedControl() {
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        driveMotorTargetSpeed[0] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[1] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        driveMotorTargetSpeed[2] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[3] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        activeOpMode.telemetry.addData("drive input", drive);
        activeOpMode.telemetry.addData("strafe input", strafe);
        activeOpMode.telemetry.addData("turn input", turn);

        for (int i = 0; i < 4; i++ ){
            driveMotor[i].setVelocityPIDFCoefficients(P,I,D,F);
            driveMotor[i].setVelocity(driveMotorTargetSpeed[i]);
            //activeOpMode.telemetry.addData("Motor " + i + " Target Speed", Math.round(100.0 * driveMotorTargetSpeed[i] / TICKS_PER_REV));
            //activeOpMode.telemetry.addData("Actual Motor Speed", Math.round(100.0 * driveMotor[i].getVelocity() / TICKS_PER_REV));
            // add if need to set PID:
        }
    }

    public void mecanumDrivePowerControl (){
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

        activeOpMode.telemetry.addData("drive input", drive);
        activeOpMode.telemetry.addData("strafe input", strafe);
        activeOpMode.telemetry.addData("turn input", turn);

        for (int i = 0; i < 4; i++ ) {
            driveMotor[i].setPower(driveMotorPower[i]);
//            caption = "Motor " + i + " Power";
//            activeOpMode.telemetry.addData(caption, Math.round(100.0 * driveMotorPower[i])/100.0);
//            activeOpMode.telemetry.addData("Actual Motor Speed", Math.round(100.0 * driveMotor[i].getVelocity() / TICKS_PER_REV));
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

    public void setDriveSpeedFactor(double factor) {
        autoDriveSpeedFactor = factor;
    }

    public void setStrafeSpeedFactor(double factor) {
        autoStrafeSpeedFactor = factor;
    }

    public void setTurnSpeedFactor(double factor) {
        autoTurnSpeedFactor = factor;
    }

    public void setBackdropSafetyZone(boolean b) {
        backdropSafetyZoneFlag = b;
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
}
