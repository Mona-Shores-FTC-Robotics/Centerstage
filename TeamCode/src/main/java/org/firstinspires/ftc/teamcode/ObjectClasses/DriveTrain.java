package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class DriveTrain {
    // DriveTrain tuning constants
    final double P = 1; // default = 10
    final double D = 0; // default = 0
    final double I = 0; // default = 3
    final double F = 0; // default = 0
    private final double STRAFE_FACTOR = 1.2;
    private final double STARTING_RAMP_VALUE = 1.0;
    private final double RAMP_INCREMENT = 0.0;
    private final double MAX_TIP = 10.0;
    private final double STICK_DEAD_ZONE = .1;
    private final double AUTO_TURN_MAX_SPEED = 1.0;
    private final double ALLOWED_TURN_ERROR = .5;
    private final double TURN_TO_ANGLE_FACTOR = 1.0 / 180.0;

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
    public double driveMotorPower[] = {0.0,0.0,0.0,0.0};
    public double driveMotorTargetSpeed[] = {0.0,0.0,0.0,0.0};
    public int driveMotorTargetPosition[] = {0,0,0,0};

    public double drive = 0.0;
    public double strafe = 0.0;
    public double turn = 0.0;

    private boolean manualDriveControlFlag = true;

    /* Private OpMode objects and variables */
    private double ramp = STARTING_RAMP_VALUE;
    private String runningToPosition = "Not Running";
    private boolean tipRecoveryFlag = false;
    private boolean turning = false;
    private double targetAngle = 0.0;
    private double angleDelta = 0.0;

    private final ElapsedTime runToPositionPeriod = new ElapsedTime();
    private LinearOpMode activeOpMode;
    private HardwareMap hwMap = null;

    Gamepad driverGamepad;



    /* Constructor */
    public DriveTrain() {

    }

    /* METHOD: Initialize Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        activeOpMode = Robot.getInstance().getActiveOpMode();

        //set the driverGamepad so we can use the shortname throughout this class
        driverGamepad = Robot.getInstance().getActiveOpMode().gamepad1;

        // Manual Control is allowed at init
        setManualDriveControlFlag(true);

        // Define and Initialize Motors
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rvrs = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction driveMotorDirections[] = {rvrs, fwd, rvrs, fwd};

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = hwMap.get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setDirection(driveMotorDirections[i]);
        }

    }

    public void drive(){
        Gamepad driverGamepad = Robot.getInstance().getActiveOpMode().gamepad1;

        double driveInput = -driverGamepad.left_stick_y;
        double strafeInput = driverGamepad.left_stick_x;
        double turnInput = driverGamepad.right_stick_x;

        //Check if the driver sticks are being moved at all so we can cancel any automated driving
        if (    Math.abs(driveInput) > STICK_DEAD_ZONE ||
                Math.abs(strafeInput) > STICK_DEAD_ZONE ||
                Math.abs(turnInput) > STICK_DEAD_ZONE)
        {
            setManualDriveControlFlag(true);
            setDrive(driveInput);
            setTurn(turnInput);
            setStrafe(strafeInput);

            //call the drive function with the drive/turn/strafe values set based on the driver controls
            mecanumDriveSpeedControl();

        } else if (!getManualDriveControlFlag()) {
            //call the drive function with the drive/turn/strafe values that are already set by vision (or some other system)
            mecanumDriveSpeedControl();
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            drive = 0;
            strafe = 0;
            turn = 0;
            mecanumDriveSpeedControl();
        }
    }

    /** fieldOrientedControl takes inputs from the gamepad and the angle of the robot.
     * These inputs are used to calculate the drive, strafe and turn inputs needed for the MecanumDrive method.
     */
    public void fieldOrientedControl (double forward, double sideways, double turning, double robotAngle){  // remove turning input
        // Consider moving these constants to top of class
        robotAngle = robotAngle * Math.PI / 180;
        double DRIVE_SPEED_FACTOR = 1.0;
        double STRAFE_SPEED_FACTOR = 1.0;
        double TURN_SPEED_FACTOR = 1.0; // delete line

        double magnitude = Math.sqrt(Math.pow(forward, 2) + Math.pow(sideways, 2));
        double driveAngle = Math.copySign(Math.acos(forward/magnitude), Math.asin(-sideways));
        double deltaAngle = robotAngle-driveAngle;

        drive = DRIVE_SPEED_FACTOR * magnitude * Math.cos(deltaAngle);
        strafe = STRAFE_SPEED_FACTOR * magnitude * Math.sin(deltaAngle);
        turn = TURN_SPEED_FACTOR * turning; // Delete line

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
            driveMotor[i].setVelocity(driveMotorTargetSpeed[i]);
            activeOpMode.telemetry.addData("Motor " + i + " Target Speed", Math.round(100.0 * driveMotorTargetSpeed[i] / TICKS_PER_REV));
            activeOpMode.telemetry.addData("Actual Motor Speed", Math.round(100.0 * driveMotor[i].getVelocity() / TICKS_PER_REV));
        }
    }


    // turns to a specified angle based on driver inputs.
    public  boolean turnToAngleCalc(boolean right, boolean fwd, boolean left, boolean back, LinkedList<Double> robotAngle){
        if (fwd) {targetAngle = 0;}
        else if (left) {targetAngle = 90;}
        else if (back) {targetAngle = 180;}
        else if (right) {targetAngle = -90;}

        angleDelta = targetAngle - robotAngle.get(0);
        if (angleDelta > 180) {angleDelta -= 360;}
        else if (angleDelta < -180) {angleDelta += 360;}

        if (Math.abs(angleDelta) > ALLOWED_TURN_ERROR){ // convert to else if
            turn = - angleDelta * TURN_TO_ANGLE_FACTOR * AUTO_TURN_MAX_SPEED;
            drive = 0;
            strafe = 0;

            return true;
        }
        else {return false;}
    }

    public void setDrive(double input_drive) {
        drive = input_drive;
    }

    public void setStrafe(double input_strafe) {
        strafe = input_strafe;
    }

    public void setTurn(double input_turn) {
        strafe = input_turn;
    }

    public void setManualDriveControlFlag(boolean flag) {
        manualDriveControlFlag = flag;
    }

    public boolean getManualDriveControlFlag() {
        return manualDriveControlFlag;
    }

}
