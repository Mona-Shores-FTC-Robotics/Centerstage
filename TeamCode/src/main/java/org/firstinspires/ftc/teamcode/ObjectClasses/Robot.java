package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.EndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.LiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.TestIntake;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision;

public class Robot {

    private static Robot robot = null;

    private static ElapsedTime teleOpRuntime;
    private static LinearOpMode activeOpMode;
    private static HardwareMap hardwareMap;

    private static DriveTrain drivetrain;
    private static Arm arm;
    private static LiftSlide liftSlide;
    private static Gyro gyro;
    private static EndEffector endEffector;
    private static Vision vision;
    private static Telemetry telemetry;
    private static TestIntake testIntake;

    /* Constructor */
    private Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        teleOpRuntime = new ElapsedTime();

        drivetrain = new DriveTrain();
        liftSlide = new LiftSlide();
        arm = new Arm();
        gyro = new Gyro();
        endEffector = new EndEffector();
        vision = new Vision();
        testIntake = new TestIntake();

    }

    public static Robot createInstance(LinearOpMode opMode) {
        robot = new Robot(opMode);
        return robot;
    }

    public void initialize(HardwareMap hwMap) {

        switch (Constants.getRobot()) {
            case ROBOT_CENTERSTAGE:
            {
                vision.init();
                gyro.init();
                drivetrain.init();
                liftSlide.init();  //Uses a motor
                arm.init(); // Uses a servo
                endEffector.init(); //Uses a servo
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
                vision.init();
                drivetrain.init();
                gyro.init();
                break;
            }
            case ROBOT_MOTOR_TEST_MECHANISM: {
                testIntake.init();
                break;
            }
            case ROBOT_ARM_END_EFFECTOR:
            {
                vision.init();
                gyro.init();
                drivetrain.init();
                arm.init(); // Uses a servo
                endEffector.init(); //Uses a servo
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
            telemetry.addLine("error");
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
    public Gyro getGyro()  {return gyro;    }
    public DriveTrain getDrivetrain()  {return drivetrain;}
    public Vision getVision()  {return vision;}
    public EndEffector getEndEffector()  {return endEffector;}
    public TestIntake getTestIntake()  {return testIntake;}

}

