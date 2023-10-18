package org.firstinspires.ftc.teamcode.ObjectClasses.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

public class DriveController {

    public double controllerDrive = 0.0;
    public double controllerStrafe = 0.0;
    public double controllerTurn = 0.0;

    public double aprilTagDrive = 0.0;
    public double aprilTagStrafe = 0.0;
    public double aprilTagTurn = 0.0;

    public boolean fieldOrientedControlFlag = true;
    public boolean lockedHeadingFlag = false;
    public boolean drivingToAprilTag = false;

    private LinearOpMode activeOpMode;
    private Gamepad driverGamepad;

    private boolean autoTurning=false;
    private double currentTurnError;
    private double autoTurn;
    private double turnDegrees;

    private InitVisionProcessor initVisionProcessor;
    private Vision vision;
    private MecanumDriveMona mecanumDrive;

    /* Constructor */
    public DriveController() {
    }

    /* METHOD: Initialize Hardware interfaces */
    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        driverGamepad = GamepadHandling.getCurrentDriverGamepad();
        initVisionProcessor = Robot.getInstance().getVision().getInitVisionProcessor();
        vision = Robot.getInstance().getVision();
        mecanumDrive = Robot.getInstance().getMecanumDriveMona();
    }

    public void setDriveStrafeTurnValues(){
        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive() || drivingToAprilTag) {
            //Store the adjusted gamepad values as drive/strafe/turn
            driverGamepad = GamepadHandling.getCurrentDriverGamepad();
            controllerDrive = -driverGamepad.left_stick_y * mecanumDrive.MotorParameters.DRIVE_SPEED_FACTOR;
            controllerStrafe = driverGamepad.left_stick_x * mecanumDrive.MotorParameters.STRAFE_SPEED_FACTOR;
            controllerTurn = driverGamepad.right_stick_x * mecanumDrive.MotorParameters.TURN_SPEED_FACTOR;

            //Check if we are turning automatically using Turn or TurnTo and change the turn value if we are
            if (lockedHeadingFlag == true) {
                turnTo(0);
            }
            if (autoTurning) {
                turnUpdate();
                controllerTurn = autoTurn;
            }

            //Check if we are using field oriented control and change the drive/strafe values if we are
            if (fieldOrientedControlFlag == true) {
                fieldOrientedControl();
            }



            // Cancel AprilTag driving if the driver is moving away from the backdrop
            if (controllerDrive < -.1) drivingToAprilTag = false;

            // Cancel AprilTag driving if the driver is moving the stick away from the backdrop (or strafing or turning)
            //if (controllerDrive < -.1 || controllerStrafe > .1 || controllerStrafe < -.1 || controllerTurn <-.1 || controllerTurn > .1) drivingToAprilTag = false;

            //Aligning to the Backdrop AprilTags - CASE RED
            if (Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    vision.redBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                vision.AutoDriveToBackdropRed();
                controllerDrive = aprilTagDrive;
                controllerStrafe = aprilTagStrafe;
                controllerTurn = aprilTagTurn;
                drivingToAprilTag = true;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    vision.blueBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                vision.AutoDriveToBackdropBlue();
                controllerDrive = aprilTagDrive;
                controllerStrafe = aprilTagStrafe;
                controllerTurn = aprilTagTurn;
                drivingToAprilTag = true;
            } else drivingToAprilTag = false;
        }
        else if (autoTurning) {
            turnUpdate();
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = autoTurn;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = 0;
        }
        mecanumDrive.drive = controllerDrive;
        mecanumDrive.strafe = controllerStrafe;
        mecanumDrive.turn = controllerTurn;
    }

    public void fieldOrientedControl (){
        double y = controllerDrive;
        double x = controllerStrafe;
        double botHeading = Robot.getInstance().getGyro().currentAbsoluteYawRadians;

        // Rotate the movement direction counter to the bot's rotation
        controllerStrafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        controllerDrive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        controllerStrafe = Math.min( controllerStrafe * 1.1, 1);  // Counteract imperfect strafing
    }

    public void turn (double degrees) {
        Robot.getInstance().getGyro().resetRelativeYaw();
        currentTurnError = degrees;
        turnDegrees = degrees;
        autoTurning = true;
    }

    public void turnUpdate () {
        if (Math.abs(currentTurnError) > 2){
            double motorPower = (currentTurnError < 0 ? 0.5 : -0.5);
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
