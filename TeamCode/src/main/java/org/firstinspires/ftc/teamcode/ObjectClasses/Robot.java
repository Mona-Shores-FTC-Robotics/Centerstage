package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dalvik.system.DelegateLastClassLoader;

public class Robot {

    private static Robot robot = null;

    private static ElapsedTime teleOpRuntime;
    private static LinearOpMode activeOpMode;
    private static HardwareMap hardwareMap;

    private static DriveTrain drivetrain;
    private static Arm arm;
    private static Lift lift;
    private static Gyro gyro;
    private static IntakeOuttake intake;
    private static Vision vision;
    private static GamepadHandling gamepadHandling;
    private static Telemetry telemetry;

    /* Constructor */
    private Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        teleOpRuntime = new ElapsedTime();

        drivetrain = new DriveTrain();
        lift = new Lift();
        arm = new Arm();
        gyro = new Gyro();
        intake = new IntakeOuttake();
        vision = new Vision();
        gamepadHandling = new GamepadHandling();
        //testIntake = new TestIntake();

    }

    public static synchronized Robot createInstance(LinearOpMode opMode) {
        if (robot == null) {
            robot = new Robot(opMode);
        }
        return robot;
    }

    public void initialize(HardwareMap hwMap) {

        //Make the telemetry print to both the Driver Station and to the Dashboard
        //TODO investigate how this is supposed to work
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        switch (Constants.getRobot()) {
            case ROBOT_2023:
            {
                gyro.init();
                drivetrain.init();
                lift.init();
                arm.init();
                intake.init();
                break;
            }
            case ROBOT_CHASSIS:
            {
                drivetrain.init();
                gyro.init();
                break;
            }
            case ROBOT_VISION:
            {
                drivetrain.init();
                vision.init();
                gyro.init();
                break;
            }
            case ROBOT_MECHANISM:
                break;

            default:
                break;
        }
    }


    /** Getters **/
    // Static method to get single instance of Robot
    public static synchronized Robot getInstance() {
        if (robot == null) {
            //error
        }
        return robot;
    }

    public ElapsedTime getTeleOpRuntime() {
        return teleOpRuntime;
    }
    public LinearOpMode getActiveOpMode() {
        return activeOpMode;
    }
    public HardwareMap getHardwareMap() {return activeOpMode.hardwareMap;}
    public Gyro getGyro()  {
        return gyro;
    }
    public DriveTrain getDriveTrain()  {return drivetrain;}
    public Vision getVision()  {return vision;}

}

