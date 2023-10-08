package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

public class DriveTrain {
    // DriveTrain tuning constants

    private double DEFAULT_P = 11; // default = 10
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

    public double aprilTagDrive = 0.0;
    public double aprilTagStrafe = 0.0;
    public double aprilTagTurn = 0.0;

    public final double DRIVE_SPEED_FACTOR = .9;
    private final double STRAFE_SPEED_FACTOR = .9;
    private final double TURN_SPEED_FACTOR = .5;

    private double safetyDriveSpeedFactor = DRIVE_SPEED_FACTOR;

    private boolean fieldOrientedControlFlag = true;
    private boolean backdropSafetyZoneFlag = false;

    private LinearOpMode activeOpMode;
    private Gamepad driverGamepad;

    private boolean autoTurning=false;
    private double currentTurnError;
    private double autoTurn;
    private double turnDegrees;

    private InitVisionProcessor initVisionProcessor;
    private Vision vision;

    /* Constructor */
    public DriveTrain() {
    }

    /* METHOD: Initialize Hardware interfaces */
    public void init() {
        // Save reference to Hardware map
        activeOpMode = Robot.getInstance().getActiveOpMode();

        //set the driverGamepad so we can use the shortname throughout this class
        driverGamepad = GamepadHandling.getCurrentDriverGamepad();
        initVisionProcessor = Robot.getInstance().getVision().getInitVisionProcessor();

        vision = Robot.getInstance().getVision();

        // Define and Initialize Motors
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rvrs = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction driveMotorDirections[] = {rvrs, fwd, rvrs, fwd};

        P=DEFAULT_P;
        D=DEFAULT_D;
        I=DEFAULT_I;
        F=DEFAULT_F;

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = Robot.getInstance().getHardwareMap().get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setDirection(driveMotorDirections[i]);
        }
    }

    public void drive(){
        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive()) {
            //Store the adjusted gamepad values as drive/strafe/turn
            driverGamepad = GamepadHandling.getCurrentDriverGamepad();
            drive = -driverGamepad.left_stick_y * DRIVE_SPEED_FACTOR;
            strafe = driverGamepad.left_stick_x * STRAFE_SPEED_FACTOR;
            turn = driverGamepad.right_stick_x * TURN_SPEED_FACTOR;

            //Check if we are turning automatically using Turn or TurnTo and change the turn value if we are
            if (autoTurning) {
                turnUpdate();
                turn = autoTurn;
            }

            //Check if we are using field oriented control and change the drive/strafe values if we are
            if (fieldOrientedControlFlag == true) {
                fieldOrientedControl();
            }

            //Aligning to the Backdrop AprilTags - CASE RED
            if (Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    vision.redBackdropAprilTagFound &&
                    drive > .1 &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                vision.AutoDriveToBackdropRed();
                drive = aprilTagDrive;
                strafe = aprilTagStrafe;
                turn = aprilTagTurn;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    vision.blueBackdropAprilTagFound &&
                    drive > .1 &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                vision.AutoDriveToBackdropBlue();
                drive = aprilTagDrive;
                strafe = aprilTagStrafe;
                turn = aprilTagTurn;
            } else if ((vision.blueBackdropAprilTagFound || vision.redBackdropAprilTagFound) && drive > 0)
            {
                drive = Math.min(drive, safetyDriveSpeedFactor);
            }

        }

        else if (autoTurning) {
            turnUpdate();
            drive = 0;
            strafe = 0;
            turn = autoTurn;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            drive = 0;
            strafe = 0;
            turn = 0;
            for (int i = 0; i < 4; i++){
                driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        //call the drive function with the drive/turn/strafe values set based on the driver controls
        mecanumDriveSpeedControl();
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

    public void fieldOrientedControl (){
        double y = drive;
        double x = strafe;
        double botHeading = Robot.getInstance().getGyro().currentAbsoluteYawRadians;

        // Rotate the movement direction counter to the bot's rotation
        strafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        drive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        strafe = Math.min( strafe * 1.1, 1);  // Counteract imperfect strafing
    }

    public void setAprilTagDrive(double autoDrive) { aprilTagDrive = autoDrive;}
    public void setAprilTagStrafe(double autoStrafe) { aprilTagStrafe = autoStrafe;}
    public void setAprilTagTurn(double autoTurn) {  aprilTagTurn = autoTurn;  }

    public void setFieldOrientedControlFlag(boolean flag) {
        fieldOrientedControlFlag = flag;
    }

    public boolean getFieldOrientedControlFlag() {
        return fieldOrientedControlFlag;
    }

    public void setSafetyDriveSpeedFactor(double factor) { safetyDriveSpeedFactor = factor;}

    public double getSafetyDriveSpeedFactor() { return safetyDriveSpeedFactor;}

    public void setBackdropSafetyZone(boolean flag) {
        backdropSafetyZoneFlag = flag;
    }

      public void telemetryDriveTrain() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");

        activeOpMode.telemetry.addData("Drive: ", drive);
        activeOpMode.telemetry.addData("Strafe: ", strafe);
        activeOpMode.telemetry.addData("Turn: ", turn);

        for (int i = 0; i < 4; i++ ){
            double targetSpeed = Math.round(100.0 * driveMotorTargetSpeed[i] / TICKS_PER_REV);
            double actualSpeed = Math.round(100.0 * driveMotor[i].getVelocity() / TICKS_PER_REV);
            double power =  Math.round(100.0 * driveMotorPower[i])/100.0;
            activeOpMode.telemetry.addLine("Motor " + i + " Speed: " + JavaUtil.formatNumber(actualSpeed, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeed, 4, 1)  + " " + "Power: " +  Math.round(100.0 * driveMotor[i].getPower())/100.0);
        }
    }

    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    public void setMotorPower (double lF, double rF, double lB, double rB){
        driveMotor[0].setPower(lF);
        driveMotor[1].setPower(rF);
        driveMotor[2].setPower(lB);
        driveMotor[3].setPower(rB);
    }

    public void turn (double degrees) {
        Robot.getInstance().getGyro().resetRelativeYaw();
        currentTurnError = degrees;
        turnDegrees = degrees;
        autoTurning = true;
    }

    public void turnUpdate () {
        if (Math.abs(currentTurnError) > 2){
            double motorPower = (currentTurnError < 0 ? 0.7 : -0.7);
            autoTurn = motorPower;
            currentTurnError = turnDegrees - Robot.getInstance().getGyro().getCurrentRelativeYaw();
            Robot.getInstance().getActiveOpMode().telemetry.addData("error", currentTurnError);
            Robot.getInstance().getActiveOpMode().telemetry.update();
        } else {
            autoTurning = false;
            autoTurn=0;
        }
    }

    public void turnTo(double degrees) {
        double error = degrees - Robot.getInstance().getGyro().currentAbsoluteYawDegrees;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);
    }

    public void turnToPID(double degrees) {
        double error = degrees - Robot.getInstance().getGyro().currentAbsoluteYawDegrees;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);
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

        for (int i = 0; i < 4; i++ ) {
            driveMotor[i].setPower(driveMotorPower[i]);
        }
    }

}
