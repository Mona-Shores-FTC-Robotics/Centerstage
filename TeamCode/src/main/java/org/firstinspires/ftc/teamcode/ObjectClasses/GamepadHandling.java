package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadHandling {

    private static Gamepad currentDriverGamepad;
    private static Gamepad currentOperatorGamepad;
    private static Gamepad previousDriverGamepad;
    private static Gamepad previousOperatorGamepad;

    private static Gamepad driverGamepad;
    private static Gamepad operatorGamepad;

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

    public static Boolean gamepadIsActive(@NonNull Gamepad gamepad) {
        if (Math.abs(gamepad.left_stick_x) + Math.abs(gamepad.left_stick_y) + Math.abs(gamepad.right_stick_x) > .1) {
            return true;
        } else return false;
    }

    public static void DriverControls() {
        DriveTrain drivetrain = Robot.getInstance().getDriveTrain();

        //Start button toggles field oriented control
        if (currentDriverGamepad.start && !previousDriverGamepad.start) {
            if (Robot.getInstance().getDriveTrain().getFieldOrientedControlFlag()) {
                //drive normally - not in field oriented control
                drivetrain.setFieldOrientedControlFlag(false);
            } else {
                //drive in field oriented control
                Robot.getInstance().getGyro().resetYaw();
                drivetrain.setFieldOrientedControlFlag(true);
            }
        }
    }

    public static void OperatorControls() {

    // the X/Y/B buttons set the deliver location to left, center, or right
            if(currentOperatorGamepad.x && !previousOperatorGamepad.x){
        Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.LEFT);
    }
            if(currentOperatorGamepad.y && !previousOperatorGamepad.y){
        Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.CENTER);
    }
            if(currentOperatorGamepad.b && !previousOperatorGamepad.b){
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
    public static Gamepad getOperatorGamepad() {
        return currentOperatorGamepad;
    }
}
