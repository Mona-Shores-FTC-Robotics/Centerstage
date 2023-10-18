package org.firstinspires.ftc.teamcode.ObjectClasses;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ConstantTrajectoryBuilder;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Controllers.DriveController;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

import java.lang.reflect.Field;
import java.util.HashMap;

public class GamepadHandling {

    private static Gamepad currentDriverGamepad;
    private static Gamepad currentOperatorGamepad;
    private static Gamepad previousDriverGamepad;
    private static Gamepad previousOperatorGamepad;

    private static Gamepad driverGamepad;
    private static Gamepad operatorGamepad;

    private static boolean overrideAprilTagDriving = false;

    public static boolean LockedInitSettingsFlag = false;
    public static boolean ManualOverrideInitSettingsFlag = false;
    private static HashMap<String, Integer> buttonMap = new HashMap<String, Integer>();

    private static MecanumDriveMona mecanumDrive;
    private static Telemetry telemetry;

    private static Gamepad.RumbleEffect endGameRumbleEffect;
    private static Gamepad.RumbleEffect
            problemRumbleEffect;
    private static Gamepad.LedEffect problemLedEffect;

    private static int timeoutRumbleCounter;

    public GamepadHandling() {

    }

    public static void init() {
        currentDriverGamepad = new Gamepad();
        currentOperatorGamepad = new Gamepad();
        previousDriverGamepad = new Gamepad();
        previousOperatorGamepad = new Gamepad();

        driverGamepad = Robot.getInstance().getActiveOpMode().gamepad1;
        operatorGamepad = Robot.getInstance().getActiveOpMode().gamepad2;

        mecanumDrive = Robot.getInstance().getMecanumDriveMona();
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;


        //driver is (not sure what color this will be)
        Robot.getInstance().getActiveOpMode().gamepad1.setLedColor(0,.2,.4,LED_DURATION_CONTINUOUS );


        //operator is white
        Robot.getInstance().getActiveOpMode().gamepad2.setLedColor(1,1,1,LED_DURATION_CONTINUOUS );


        endGameRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();

        problemRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
                .addStep(.5, .5, 250)  //  Rumble both motors 50% for 250 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
                .build();

        problemLedEffect = new Gamepad.LedEffect.Builder()
                .addStep(0, 1, 0, 500) // Show green for 250ms
                .addStep(0, 0, 0, 500) // Show white for 250ms
                .addStep(0, 1, 0, LED_DURATION_CONTINUOUS) // Show white for 250ms
                .build();

        //set the rumble counter to 0
        timeoutRumbleCounter=0;
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
        if     (Math.abs(GamepadHandling.getCurrentDriverGamepad().left_stick_x) > .1 ||
                Math.abs(GamepadHandling.getCurrentDriverGamepad().left_stick_y) > .1 ||
                Math.abs(GamepadHandling.getCurrentDriverGamepad().right_stick_x) > .1 ){
            return true;
        } else return false;
    }

    public static void DriverControls() {
        DriveController driveController = Robot.getInstance().getDriveController();

        //Start button toggles field oriented control
        if (currentDriverGamepad.start && !previousDriverGamepad.start) {
            if (driveController.fieldOrientedControlFlag==true) {
                //drive normally - not in field oriented control
                driveController.fieldOrientedControlFlag = false;
            } else {
                //drive in field oriented control
                driveController.fieldOrientedControlFlag = true;
            }
        }

        if (currentDriverGamepad.left_bumper)
        {
            overrideAprilTagDriving = true;
        } else overrideAprilTagDriving =false;

        if (currentDriverGamepad.right_bumper)
        {
            Robot.getInstance().getDriveController().lockedHeadingFlag = true;
        } else Robot.getInstance().getDriveController().lockedHeadingFlag = false;



        //Reset Gyro Button
        if (GamepadHandling.driverButtonPressed("x")){
            Robot.getInstance().getGyro().resetAbsoluteYaw();
        }

        if (GamepadHandling.driverButtonPressed("y"))
        {
//            runBlocking(Robot.getInstance().getMecanumDriveMona().actionBuilder(mecanumDrive.pose)
//                    .strafeTo(new Vector2d(mecanumDrive.pose.position.x+6, mecanumDrive.pose.position.y))
//                    .build());
            telemetry.addLine("y");
        }

        if (GamepadHandling.driverButtonPressed("a"))
        {
//            runBlocking(mecanumDrive.actionBuilder(mecanumDrive.pose)
//                    .strafeTo(new Vector2d(mecanumDrive.pose.position.x-6, mecanumDrive.pose.position.y))
//                    .build());
            telemetry.addLine("a");
        }

        if (GamepadHandling.getCurrentDriverGamepad().b)
        {
            telemetry.addLine("b");



        }

        if (GamepadHandling.getCurrentDriverGamepad().dpad_up)
        {
            telemetry.addLine("d-pad up");
        }
        if (GamepadHandling.getCurrentDriverGamepad().dpad_down)
        {
            telemetry.addLine("d-pad down");
        }
        if (GamepadHandling.getCurrentDriverGamepad().dpad_left)
        {
            telemetry.addLine("d-pad left");
        }
        if (GamepadHandling.getCurrentDriverGamepad().dpad_right)
        {
            telemetry.addLine("d-pad right");
        }

        //this button is labeled as "share" on our gamepad
        if (GamepadHandling.getCurrentDriverGamepad().back)
        {
            telemetry.addLine("back");
        }

        if (GamepadHandling.getCurrentDriverGamepad().ps)
        {
            telemetry.addLine("ps");
        }

        if (GamepadHandling.getCurrentDriverGamepad().touchpad_finger_1)
        {
            telemetry.addLine("touchpad_finger_1");
        }


        if (GamepadHandling.getCurrentDriverGamepad().touchpad_finger_2)
        {
            telemetry.addLine("touchpad_finger_2");
        }

        //these dont work
        if (GamepadHandling.getCurrentDriverGamepad().share)
        {
            telemetry.addLine("share");
        }

        if (GamepadHandling.getCurrentDriverGamepad().options)
        {
            telemetry.addLine("options");
        }

        if (GamepadHandling.getCurrentDriverGamepad().guide)
        {
            telemetry.addLine("guide");
        }

        if (GamepadHandling.getCurrentDriverGamepad().touchpad)
        {
            telemetry.addLine("touchpad");
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

        if (LockedInitSettingsFlag)
        {
            telemetry.addLine("Press B to unlock Alliance Color and Side of Field");
            if (GamepadHandling.getCurrentDriverGamepad().b && !GamepadHandling.getPreviousDriverGamepad().b)
            {
                LockedInitSettingsFlag = false;
            }
        } else if (!LockedInitSettingsFlag)
        {
            if (ManualOverrideInitSettingsFlag)
            {
                initVisionProcessor.allianceColorFinal = initVisionProcessor.allianceColorOverride;
                initVisionProcessor.sideOfFieldFinal = initVisionProcessor.sideOfFieldOverride;
                initVisionProcessor.teamPropLocationFinal = initVisionProcessor.teamPropLocationOverride;
            }
            telemetry.addLine("Lock with B");
            telemetry.addLine( initVisionProcessor.allianceColorFinal + " " + initVisionProcessor.sideOfFieldFinal + " " + initVisionProcessor.teamPropLocationFinal);

            if (GamepadHandling.getCurrentDriverGamepad().b && !GamepadHandling.getPreviousDriverGamepad().b)
            {
                LockedInitSettingsFlag = true;
            }

            if (!ManualOverrideInitSettingsFlag) {
                telemetry.addLine("Override with A");
                if (GamepadHandling.getCurrentDriverGamepad().a && !GamepadHandling.getPreviousDriverGamepad().a) {
                    ManualOverrideInitSettingsFlag = true;
                }
            } else if (ManualOverrideInitSettingsFlag) {
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
                    ManualOverrideInitSettingsFlag = false;
                }
            }
        }
    }

    public static boolean driverButtonPressed(String buttonName) {
        Field currentField= null;
        try {
            currentField = currentDriverGamepad.getClass().getField(buttonName);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        Field previousField = null;
        try {
            previousField = previousDriverGamepad.getClass().getField(buttonName);
        } catch ( NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        boolean currentButton = false;
        try {
                 currentButton = (boolean) currentField.getBoolean(currentDriverGamepad);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
         }
        boolean previousButton = false;

        try {
            previousButton = (boolean) previousField.getBoolean(previousDriverGamepad);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        return (currentButton && !previousButton);
    }

    public static boolean operatorButtonPressed(String buttonName) {
        Field currentField= null;
        try {
            currentField = currentOperatorGamepad.getClass().getField(buttonName);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        Field previousField = null;
        try {
            previousField = previousOperatorGamepad.getClass().getField(buttonName);
        } catch ( NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        boolean currentButton = false;
        try {
            currentButton = (boolean) currentField.getBoolean(currentOperatorGamepad);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        boolean previousButton = false;

        try {
            previousButton = (boolean) previousField.getBoolean(previousOperatorGamepad);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        return (currentButton && !previousButton);
    }

    public static void endGameRumble() {
        Robot.getInstance().getActiveOpMode().gamepad1.runRumbleEffect(endGameRumbleEffect);
        Robot.getInstance().getActiveOpMode().gamepad2.runRumbleEffect(endGameRumbleEffect);
    }

    public static void setRed() {
        //set driver gamepad to red
        Robot.getInstance().getActiveOpMode().gamepad1.setLedColor(1, 0, 0,  LED_DURATION_CONTINUOUS );
        //Robot.getInstance().getActiveOpMode().gamepad2.setLedColor(1, 0, 0,  LED_DURATION_CONTINUOUS);
    }


    public static void setBlue() {
        //set driver gamepad to blue
        Robot.getInstance().getActiveOpMode().gamepad1.setLedColor(0, 0, 1,  LED_DURATION_CONTINUOUS);
        //Robot.getInstance().getActiveOpMode().gamepad2.setLedColor(0, 0, 1,  LED_DURATION_CONTINUOUS);
    }

    public static void problemInInitRumble() {

        //only do the rumble 5 times so we don't burn out the rumble motors
        if  (!Robot.getInstance().getActiveOpMode().gamepad1.isRumbling() && timeoutRumbleCounter < 5) {
            timeoutRumbleCounter+=1;
            Robot.getInstance().getActiveOpMode().gamepad1.runRumbleEffect(problemRumbleEffect);
            //Robot.getInstance().getActiveOpMode().gamepad2.runRumbleEffect(problemRumbleEffect);
        }
    }

    public static void problemInInitLed() {
            Robot.getInstance().getActiveOpMode().gamepad1.setLedColor(0,1,0, LED_DURATION_CONTINUOUS);
            //Robot.getInstance().getActiveOpMode().gamepad2.setLedColor(0,1,0, LED_DURATION_CONTINUOUS);
        }
}
