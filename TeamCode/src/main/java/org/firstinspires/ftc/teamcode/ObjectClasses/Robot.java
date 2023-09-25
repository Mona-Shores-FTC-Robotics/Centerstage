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
    private static Vision vision;

    /* Constructor */
    private Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        runtime = new ElapsedTime();

        drivetrain = new DriveTrain();
        lift = new Lift();
        arm = new Arm();
        gyro = new Gyro();
        intake = new IntakeOuttake();
        vision = new Vision();
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
                drivetrain.init();
                lift.init();
                arm.init();
                gyro.init();
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
    public HardwareMap getHardwareMap() {return activeOpMode.hardwareMap;}
    public Gyro getGyro()  {
        return gyro;
    }
    public DriveTrain getDriveTrain()  {return drivetrain;}
    public Vision getVision()  {return vision;}

}

