package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

public class GamepadHandling {

    private static Gamepad currentDriverGamepad;
    private static Gamepad currentOperatorGamepad;
    private static Gamepad previousDriverGamepad;
    private static Gamepad previousOperatorGamepad;

    private static Gamepad driverGamepad;
    private static Gamepad operatorGamepad;

    public static double motorFwd = 0.0;
    public static double motorRev = 0.0;

    private static boolean overrideAprilTagDriving = false;

    public static boolean LockedFlag = false;
    public static boolean ManualOverrideFlag = false;

    public GamepadHandling() {

    }

    public static void init() {
        currentDriverGamepad = new Gamepad();
        currentOperatorGamepad = new Gamepad();
        previousDriverGamepad = new Gamepad();
        previousOperatorGamepad = new Gamepad();

        driverGamepad = Robot.getInstance().getActiveOpMode().gamepad1;
        operatorGamepad = Robot.getInstance().getActiveOpMode().gamepad2;
    }

    @NonNull
    public static Gamepad copy(@NonNull Gamepad gamepad) {
        Gamepad pad = new Gamepad();
        pad.a = gamepad.a;
        pad.b = gamepad.b;
        pad.x = gamepad.x;
        pad.y = gamepad.y;

        pad.right_bumper = gamepad.right_bumper;
        pad.left_bumper = gamepad.left_bumper;

        pad.right_trigger = gamepad.right_trigger;
        pad.left_trigger = gamepad.left_trigger;

        pad.left_stick_x = gamepad.left_stick_x;
        pad.left_stick_y = gamepad.left_stick_y;
        pad.left_stick_button = gamepad.left_stick_button;

        pad.right_stick_x = gamepad.right_stick_x;
        pad.right_stick_y = gamepad.right_stick_y;
        pad.right_stick_button = gamepad.right_stick_button;

        pad.start = gamepad.start;
        pad.back = gamepad.back;

        pad.dpad_up = gamepad.dpad_up;
        pad.dpad_left = gamepad.dpad_left;
        pad.dpad_right = gamepad.dpad_right;
        pad.dpad_down = gamepad.dpad_down;

        return pad;
    }

    public static Boolean driverGamepadIsActive() {
        if (    (Math.abs(GamepadHandling.getCurrentDriverGamepad().left_stick_x) +
                Math.abs(GamepadHandling.getCurrentDriverGamepad().left_stick_y) +
                Math.abs(GamepadHandling.getCurrentDriverGamepad().right_stick_x)) > .1) {
            return true;
        } else return false;
    }

    public static void DriverControls() {
        DriveTrain drivetrain = Robot.getInstance().getDrivetrain();

        //Start button toggles field oriented control
        if (currentDriverGamepad.start && !previousDriverGamepad.start) {
            if (Robot.getInstance().getDrivetrain().getFieldOrientedControlFlag()) {
                //drive normally - not in field oriented control
                drivetrain.setFieldOrientedControlFlag(false);
            } else {
                //drive in field oriented control
                drivetrain.setFieldOrientedControlFlag(true);
            }
        }

        //Options button resets the Yaw
        if (currentDriverGamepad.x && !previousDriverGamepad.x) {
            Robot.getInstance().getGyro().resetAbsoluteYaw();
        }

        if (currentDriverGamepad.left_bumper)
        {
            overrideAprilTagDriving = true;
        } else overrideAprilTagDriving =false;

//        if (Robot.getInstance().getVision().noVisibleTags && (currentDriverGamepad.left_bumper && !previousDriverGamepad.left_bumper)) {
//            Robot.getInstance().getDriveTrain().turnTo(105);
//        }
//
//        if (Robot.getInstance().getVision().noVisibleTags && (currentDriverGamepad.right_bumper && !previousDriverGamepad.right_bumper)) {
//            Robot.getInstance().getDriveTrain().turnTo(90);
//        }

    }

    public static void OperatorControls() {

    // the X/Y/B buttons set the deliver location to left, center, or right
            if(currentOperatorGamepad.x && !previousOperatorGamepad.x){
        Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.LEFT);
    }
            if(currentOperatorGamepad.y && !previousOperatorGamepad.y){
        Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.CENTER);
    }
            if(currentOperatorGamepad.b && !previousOperatorGamepad.b) {
                Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.RIGHT);
    }
}

    public static void storeGamepadValuesFromLastLoop() {
        previousDriverGamepad = GamepadHandling.copy(currentDriverGamepad);
        previousOperatorGamepad = GamepadHandling.copy(currentOperatorGamepad);
    }

    public static void storeCurrentGamepadValues() {
        currentDriverGamepad = GamepadHandling.copy(driverGamepad);
        currentOperatorGamepad = GamepadHandling.copy(operatorGamepad);
    }

    public static Gamepad getCurrentDriverGamepad() {
        return currentDriverGamepad;
    }
    public static Gamepad getCurrentOperatorGamepad() {
        return currentOperatorGamepad;
    }

    public static Gamepad getPreviousDriverGamepad() {
        return previousDriverGamepad;
    }
    public static Gamepad getPreviousOperatorGamepad() {
        return previousOperatorGamepad;
    }

    public static boolean getOverrideAprilTagDriving() {
        return overrideAprilTagDriving;
    }


    public static void lockColorAndSide() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        InitVisionProcessor initVisionProcessor = Robot.getInstance().getVision().getInitVisionProcessor();
        telemetry.addLine("");

        if (LockedFlag)
        {
            telemetry.addLine("Press B to unlock Alliance Color and Side of Field");
            if (GamepadHandling.getCurrentDriverGamepad().b && !GamepadHandling.getPreviousDriverGamepad().b)
            {
                LockedFlag = false;
            }
        } else if (!LockedFlag)
        {
            if (ManualOverrideFlag)
            {
                initVisionProcessor.allianceColorFinal = initVisionProcessor.allianceColorOverride;
                initVisionProcessor.sideOfFieldFinal = initVisionProcessor.sideOfFieldOverride;
                initVisionProcessor.teamPropLocationFinal = initVisionProcessor.teamPropLocationOverride;
            }
            telemetry.addLine("Lock with B");
            telemetry.addLine( initVisionProcessor.allianceColorFinal + " " + initVisionProcessor.sideOfFieldFinal + " " + initVisionProcessor.teamPropLocationFinal);

            if (GamepadHandling.getCurrentDriverGamepad().b && !GamepadHandling.getPreviousDriverGamepad().b)
            {
                LockedFlag = true;
            }

            if (!ManualOverrideFlag) {
                telemetry.addLine("Override with A");
                if (GamepadHandling.getCurrentDriverGamepad().a && !GamepadHandling.getPreviousDriverGamepad().a) {
                    ManualOverrideFlag = true;
                }
            } else if (ManualOverrideFlag) {
                telemetry.addLine("Color/Side - d-pad, Prop - bumpers");
                if (GamepadHandling.getCurrentDriverGamepad().dpad_down && !GamepadHandling.getPreviousDriverGamepad().dpad_down) {
                    initVisionProcessor.allianceColorOverride = InitVisionProcessor.AllianceColor.BLUE;
                } else if (GamepadHandling.getCurrentDriverGamepad().dpad_up && !GamepadHandling.getPreviousDriverGamepad().dpad_up) {
                    initVisionProcessor.allianceColorOverride = InitVisionProcessor.AllianceColor.RED;
                }

                if (GamepadHandling.getCurrentDriverGamepad().dpad_left && !GamepadHandling.getPreviousDriverGamepad().dpad_left) {
                    if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.BLUE) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.AUDIENCE;
                    } else if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.RED) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.BACKSTAGE;
                    }
                } else if (GamepadHandling.getCurrentDriverGamepad().dpad_right && !GamepadHandling.getPreviousDriverGamepad().dpad_right) {
                    if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.RED) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.AUDIENCE;
                    } else if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.BLUE) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.BACKSTAGE;
                    }
                }

                if (GamepadHandling.getCurrentDriverGamepad().right_bumper && !GamepadHandling.getPreviousDriverGamepad().right_bumper) {
                    if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.LEFT)
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.CENTER;
                    else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.CENTER)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.RIGHT;
                    } else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.RIGHT)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.LEFT;
                    }
                } else if (GamepadHandling.getCurrentDriverGamepad().left_bumper && !GamepadHandling.getPreviousDriverGamepad().left_bumper) {
                    if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.LEFT)
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.RIGHT;
                    else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.CENTER)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.LEFT;
                    } else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.RIGHT)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.CENTER;
                    }
                }

                telemetry.addLine("Override Off with A");
                if (GamepadHandling.getCurrentDriverGamepad().a && !GamepadHandling.getPreviousDriverGamepad().a) {
                    ManualOverrideFlag = false;
                }
            }
        }
    }

}
