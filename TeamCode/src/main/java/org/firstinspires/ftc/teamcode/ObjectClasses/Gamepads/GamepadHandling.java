package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

import java.util.HashMap;

public class GamepadHandling {

    private static GamepadEx driverGamepad;
    private static GamepadEx operatorGamepad;

    private static boolean overrideAprilTagDriving = false;
    public static boolean LockedInitSettingsFlag = false;
    public static boolean ManualOverrideInitSettingsFlag = false;
    private static HashMap<String, Integer> buttonMap = new HashMap<String, Integer>();

    private static DriveSubsystem mecanumDriveSubsystem;
    private static Telemetry telemetry;

    private static Gamepad.RumbleEffect endGameRumbleEffect;
    private static Gamepad.RumbleEffect problemRumbleEffect;
    private static Gamepad.LedEffect problemLedEffect;

    private static int timeoutRumbleCounter;

    public static ButtonReader driverAreader;
    public static ButtonReader driverBreader;

    public GamepadHandling() {

    }

    public static void init() {
        driverGamepad = new GamepadEx(Robot.getInstance().getActiveOpMode().gamepad1);
        operatorGamepad = new GamepadEx(Robot.getInstance().getActiveOpMode().gamepad2);
        Robot.getInstance().getActiveOpMode().gamepad1.setLedColor(0,.2,.4,LED_DURATION_CONTINUOUS );
        Robot.getInstance().getActiveOpMode().gamepad2.setLedColor(1,1,1,LED_DURATION_CONTINUOUS );
        CreateRumbleEffects();
        CreateLEDEffects();
    }

    private static void CreateLEDEffects() {
        problemLedEffect = new Gamepad.LedEffect.Builder()
                .addStep(0, 1, 0, 500) // Show green for 250ms
                .addStep(0, 0, 0, 500) // Show white for 250ms
                .addStep(0, 1, 0, LED_DURATION_CONTINUOUS) // Show white for 250ms
                .build();
    }

    private static void CreateRumbleEffects() {
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

        //set the rumble counter to 0
        timeoutRumbleCounter=0;
    }

    public static Boolean driverGamepadIsActive() {
        if     (Math.abs(GamepadHandling.getDriverGamepad().getLeftY()) > .1 ||
                Math.abs(GamepadHandling.getDriverGamepad().getLeftX()) > .1 ||
                Math.abs(GamepadHandling.getDriverGamepad().getRightX()) > .1 ){
            return true;
        } else return false;
    }

    public static GamepadEx getDriverGamepad() {
        return driverGamepad;
    }
    public static GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }

    public static boolean getOverrideAprilTagDriving() {
        return overrideAprilTagDriving;
    }

    public static void lockColorAndSide() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        InitVisionProcessor initVisionProcessor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor();
        telemetry.addLine("");

        if (LockedInitSettingsFlag)
        {
            telemetry.addLine("Press B to unlock Alliance Color and Side of Field");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B))
            {
                LockedInitSettingsFlag = false;
            }
        } else {
            if (ManualOverrideInitSettingsFlag)
            {
                initVisionProcessor.allianceColorFinal = initVisionProcessor.allianceColorOverride;
                initVisionProcessor.sideOfFieldFinal = initVisionProcessor.sideOfFieldOverride;
                initVisionProcessor.teamPropLocationFinal = initVisionProcessor.teamPropLocationOverride;
            }
            telemetry.addLine("Lock with B");
            telemetry.addLine( initVisionProcessor.allianceColorFinal + " " + initVisionProcessor.sideOfFieldFinal + " " + initVisionProcessor.teamPropLocationFinal);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B))
            {
                LockedInitSettingsFlag = true;
            }

            if (!ManualOverrideInitSettingsFlag) {
                telemetry.addLine("Override with A");
                if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    ManualOverrideInitSettingsFlag = true;
                }
            } else if (ManualOverrideInitSettingsFlag) {
                telemetry.addLine("Color/Side - d-pad, Prop - bumpers");
                if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    initVisionProcessor.allianceColorOverride = InitVisionProcessor.AllianceColor.BLUE;
                } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    initVisionProcessor.allianceColorOverride = InitVisionProcessor.AllianceColor.RED;
                }

                if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.BLUE) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.AUDIENCE;
                    } else if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.RED) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.BACKSTAGE;
                    }
                } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.RED) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.AUDIENCE;
                    } else if (initVisionProcessor.allianceColorOverride == InitVisionProcessor.AllianceColor.BLUE) {
                        initVisionProcessor.sideOfFieldOverride = InitVisionProcessor.SideOfField.BACKSTAGE;
                    }
                }

                if (driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.LEFT)
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.CENTER;
                    else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.CENTER)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.RIGHT;
                    } else if (initVisionProcessor.teamPropLocationOverride == InitVisionProcessor.TeamPropLocation.RIGHT)
                    {
                        initVisionProcessor.teamPropLocationOverride = InitVisionProcessor.TeamPropLocation.LEFT;
                    }
                } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
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
                if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    ManualOverrideInitSettingsFlag = false;
                }
            }
        }
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

