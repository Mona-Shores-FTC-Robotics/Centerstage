package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;

public class DriveTrain {
    // DriveTrain tuning constants
    private final double STICK_DEAD_ZONE = .1;

    private double DEFAULT_P = 8.5; // default = 10
    private double DEFAULT_D = 3; // default = 0
    private double DEFAULT_I = 0; // default = 3
    private double DEFAULT_F = 11.5; // default = 0

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

    private final double DRIVE_SPEED_FACTOR = 1;
    private final double STRAFE_SPEED_FACTOR = 1;
    private final double TURN_SPEED_FACTOR = 1;

    private double safetyDriveSpeedFactor = 1.0;
    private double safetyStrafeSpeedFactor = 1.0;
    private double safetyTurnSpeedFactor = 1.0;

    private boolean manualDriveControlFlag = true;
    private boolean fieldOrientedControlFlag = true;
    private boolean backdropSafetyZoneFlag = false;

    private LinearOpMode activeOpMode;
    private Gamepad driverGamepad;

    private boolean autoTurning=false;
    private double currentTurnError;
    private double autoTurn;
    private double turnDegrees;

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
        drive = -driverGamepad.left_stick_y;
        strafe = driverGamepad.left_stick_x;
        turn = driverGamepad.right_stick_x;

        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.gamepadIsActive(driverGamepad))
        {
            //Check if we are near the backdrop and scale our movement down if we are trying to move toward the backdrop
            setManualDriveControlFlag(true);
            if (fieldOrientedControlFlag==true)
            {
                CheckBackdropSafetyZoneFOC();
                fieldOrientedControl();
            } else {
                CheckBackdropSafetyZoneNormal();
                //call the drive function with the drive/turn/strafe values set based on the driver controls
                mecanumDriveSpeedControl();
            }
        } else if (!getManualDriveControlFlag() || autoTurning) {
            //call the drive function with the drive/turn/strafe values that are already set by vision (or some other system)
           if (!autoTurning) {
               moveRobot(aprilTagDrive, aprilTagStrafe, aprilTagTurn);
           } else if (autoTurning)
           {
               turnUpdate();
               moveRobot(aprilTagDrive, aprilTagStrafe, autoTurn);
           }

        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            drive = 0;
            strafe = 0;
            turn = 0;
            mecanumDriveSpeedControl();
            //ToDo:  Consider adding stop and reset encoders command if the drivetrain doesn't stop quickly enough.
        }
    }

    private void CheckBackdropSafetyZoneNormal() {
        //this flag is set while looking for AprilTags
        if (backdropSafetyZoneFlag)
        {
            // Only modify if the driver was trying to go forward
            // The size of the adjustments are based on how far away we are from the backdrop AprilTags
            if (drive>0) {
                drive = drive * safetyDriveSpeedFactor;
            }
        }
    }


    private void CheckBackdropSafetyZoneFOC() {
        //this flag is set while looking for AprilTags
        if (backdropSafetyZoneFlag)
        {
            // Only modify if the driver was trying to go forward
            // The size of the adjustments are based on how far away we are from the backdrop AprilTags
            if (    Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    strafe > 0) {
                strafe = strafe*safetyStrafeSpeedFactor;
            } else if (Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                strafe < 0)
            {
                strafe = strafe*safetyStrafeSpeedFactor;
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

    public void fieldOrientedControl (){

        double y = drive;
        double x = strafe;
        double rx = turn;

        double botHeading = Robot.getInstance().getGyro().currentAbsoluteYawRadians;
        //ToDo:  Move this out of the drive code, should read all sensors at the beginning of teleop loops.

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // ToDo: Once we have calculated the drive and strafe values, why don't we just plug them
        //  into the exiting speed or power control methods instead of using a unique method.

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;

        driveMotorPower[0] = leftFrontPower;
        driveMotorPower[1] = rightFrontPower;
        driveMotorPower[2] = leftBackPower;
        driveMotorPower[3] = rightBackPower;

        for (int i = 0; i < 4; i++ ) {
            driveMotor[i].setPower(driveMotorPower[i]);
        }
    }

    public void setAutoDrive(double autoDrive) { aprilTagDrive = autoDrive;}
    public void setAutoStrafe(double autoStrafe) { aprilTagStrafe = autoStrafe;}
    public void setAutoTurn(double autoTurn) {  aprilTagTurn = autoTurn;  }

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
            double motorPower = (currentTurnError < 0 ? -0.3 : 0.3);
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

}
