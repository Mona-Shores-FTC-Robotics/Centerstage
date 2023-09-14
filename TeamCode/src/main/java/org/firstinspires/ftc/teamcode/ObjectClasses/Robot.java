package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dalvik.system.DelegateLastClassLoader;

public class Robot {

    private static Robot robot = null;

    private static ElapsedTime runtime;
    private static LinearOpMode activeOpMode;
    private static HardwareMap hardwareMap;

    private static DriveTrain drivetrain;
    private static Arm arm;
    private static Lift lift;
    private static Gyro gyro;
    private static IntakeOuttake intake;

    /* Constructor */
    private Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
        hardwareMap = activeOpMode.hardwareMap;
        runtime = new ElapsedTime();

        switch (Constants.getRobot()) {
            case ROBOT_2023:
            {
                drivetrain = new DriveTrain();
                lift = new Lift();
                arm = new Arm();
                gyro = new Gyro();
                intake = new IntakeOuttake();
                break;
            }
            case ROBOT_CHASSIS:
            {
                drivetrain = new DriveTrain();
                break;
            }
            default:
                break;
        }
    }

    public static synchronized Robot createInstance(LinearOpMode opMode) {
        if (robot == null) {
            robot = new Robot(opMode);
        }
        return robot;
    }


    public void initialize(HardwareMap hwMap) {
        runtime.reset();
        switch (Constants.getRobot()) {
            case ROBOT_2023:
            {
                drivetrain.init(hwMap);
                lift.init();
                arm.init();
                gyro.init();
                intake.init();
                break;
            }
            case ROBOT_CHASSIS:
            {
                drivetrain.init(hwMap);
                break;
            }
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

    public ElapsedTime getRuntime() {
        return runtime;
    }
    public LinearOpMode getActiveOpMode() {
        return activeOpMode;
    }
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public Gyro getGyro()  {
        return gyro;
    }
    public DriveTrain getDriveTrain()  {return drivetrain;}

}
