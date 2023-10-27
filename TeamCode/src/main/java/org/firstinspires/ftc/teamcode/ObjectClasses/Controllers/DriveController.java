package org.firstinspires.ftc.teamcode.ObjectClasses.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

public class DriveController {

    public static class TURN_PARAMS {
        public static double TURN_P = .01;
        public static double TURN_I = 0;
        public static double TURN_D = 0;
        public static double TURN_F = .1;
    }


    public static DriveController.TURN_PARAMS turnParams = new DriveController.TURN_PARAMS();
    private TurnPIDController pid;
    private double targetAngle;

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
    private VisionSubsystem visionSubsystem;
    private DriveSubsystem mecanumDriveSubsystem;

    /* Constructor */
    public DriveController() {
    }

    /* METHOD: Initialize Hardware interfaces */
    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        driverGamepad = GamepadHandling.getCurrentDriverGamepad();
        initVisionProcessor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor();
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        mecanumDriveSubsystem = Robot.getInstance().getDriveSubsystem();
    }

    public void setDriveStrafeTurnValues(){
        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive() || drivingToAprilTag) {

            //Store the adjusted gamepad values as drive/strafe/turn
            driverGamepad = GamepadHandling.getCurrentDriverGamepad();
            controllerDrive = -driverGamepad.left_stick_y * mecanumDriveSubsystem.MotorParameters.DRIVE_SPEED_FACTOR;

            //more strafe deadzone
            if (Math.abs(GamepadHandling.getCurrentDriverGamepad().left_stick_x) > .2) {
                controllerStrafe = driverGamepad.left_stick_x * mecanumDriveSubsystem.MotorParameters.STRAFE_SPEED_FACTOR;
            } else controllerStrafe = 0;

            controllerTurn = driverGamepad.right_stick_x * mecanumDriveSubsystem.MotorParameters.TURN_SPEED_FACTOR;

            //Check if we are turning automatically using Turn or TurnTo and change the turn value if we are
            if (lockedHeadingFlag == true) {
                turnToPID(0);
            }

            if (autoTurning) {
                turnToPIDUpdate();
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
            if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    visionSubsystem.redBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropRed();
                controllerDrive = aprilTagDrive;
                controllerStrafe = aprilTagStrafe;
                controllerTurn = aprilTagTurn;
                drivingToAprilTag = true;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropBlue();
                controllerDrive = aprilTagDrive;
                controllerStrafe = aprilTagStrafe;
                controllerTurn = aprilTagTurn;
                drivingToAprilTag = true;
            } else drivingToAprilTag = false;
        }
        else if (autoTurning) {
            turnToPIDUpdate();
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = autoTurn;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = 0;
        }
        mecanumDriveSubsystem.drive = controllerDrive;
        mecanumDriveSubsystem.strafe = controllerStrafe;
        mecanumDriveSubsystem.turn = controllerTurn;
    }

    public void fieldOrientedControl (){
        double y = controllerDrive;
        double x = controllerStrafe;
        double botHeading = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawRadians;

        // Rotate the movement direction counter to the bot's rotation
        controllerStrafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        controllerDrive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        controllerStrafe = Math.min( controllerStrafe * 1.1, 1);  // Counteract imperfect strafing
    }


    public void turnToPID(double m_targetAngle) {
        if (!autoTurning) {
            autoTurning = true;
            pid = new TurnPIDController(m_targetAngle, turnParams.TURN_P, turnParams.TURN_I, turnParams.TURN_D, turnParams.TURN_F);
        }
        targetAngle = m_targetAngle;
    }
    public void turnToPIDUpdate() {
        if (Math.abs(targetAngle - Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees) > 1)
        {
            autoTurn = pid.updatePID(Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees);
        } else
        {
            autoTurn=0;
            autoTurning=false;
        }
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees);
    }


}
