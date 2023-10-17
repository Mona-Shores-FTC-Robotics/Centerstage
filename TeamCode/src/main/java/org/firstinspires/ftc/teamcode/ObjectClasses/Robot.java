package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Shoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Controllers.DriveController;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.EndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.LiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.TestIntake;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision;

public class Robot {

    private static Robot robot = null;

    private static ElapsedTime teleOpRuntime;
    private static LinearOpMode activeOpMode;
    private static HardwareMap hardwareMap;

    private static MecanumDriveMona mecanumDrive;
    private static Shoulder shoulder;
    private static LiftSlide liftSlide;
    private static Gyro gyro;
    private static EndEffector endEffector;
    private static Vision vision;
    private static TestIntake testIntake;

    private static DriveController driveController;

    /* Constructor */
    private Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        teleOpRuntime = new ElapsedTime();
        mecanumDrive = new MecanumDriveMona();
        liftSlide = new LiftSlide();
        shoulder = new Shoulder();
        gyro = new Gyro();
        endEffector = new EndEffector();
        vision = new Vision();
        testIntake = new TestIntake();
        driveController = new DriveController();

        //airplane launcher
        //winch
        //intake pick up
        //lights

    }

    public static Robot createInstance(LinearOpMode opMode) {
        robot = new Robot(opMode);
        return robot;
    }

    public void initialize(HardwareMap hwMap) {

        switch (RobotConstants.getRobot()) {
            case ROBOT_CENTERSTAGE:
            {
                vision.init();
                gyro.init();
                mecanumDrive.init();
                liftSlide.init();  //Uses a motor
                shoulder.init(); // Uses a servo
                endEffector.init(); //Uses a servo
                break;
            }
            case ROBOT_CHASSIS:
            {
                mecanumDrive.init();
                gyro.init();
                break;
            }
            case ROBOT_VISION:
            case ROBOT_VISION_FAST_MOTORS: {
                vision.init();
                gyro.init();
                mecanumDrive.init();
                driveController.init();
                break;
            }
            case ROBOT_MOTOR_TEST_MECHANISM: {
                testIntake.init();
                break;
            }
            case ROBOT_SHOULDER_END_EFFECTOR:
            {
                shoulder.init(); // Uses a servo
                endEffector.init(); //Uses a servo
                break;
            }
            case ROBOT_LIFT_TEST:
            {
                liftSlide.init();
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
            Robot.activeOpMode.telemetry.addLine("error");
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
    public MecanumDriveMona getMecanumDriveMona()  {return mecanumDrive;}
    public Vision getVision()  {return vision;}
    public EndEffector getEndEffector()  {return endEffector;}
    public TestIntake getTestIntake()  {return testIntake;}
    public DriveController getDriveController()  {return driveController;}
    public LiftSlide getLiftSlide()  {return liftSlide;}
    public Shoulder getShoulder()  {return shoulder;}
}



