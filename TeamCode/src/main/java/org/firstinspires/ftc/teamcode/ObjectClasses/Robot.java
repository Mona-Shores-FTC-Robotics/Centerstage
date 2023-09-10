package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot extends LinearOpMode {

    private static Robot robot = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private static LinearOpMode activeOpMode;
    private static DriveTrain drivetrain;
    private static Arm arm;
    private static Lift lift;
    private static Gyro gyro;
    private static IntakeOuttake intake;

    /* Constructor */
    private Robot() {
        activeOpMode = this;
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

    // Static method to create instance of Singleton class
    public static synchronized Robot getInstance() {
        if (robot == null) {
            robot = new Robot();
        }
        return robot;
    }

    //Method to give us access to the runtime object
    public ElapsedTime runtime() {
        return runtime;
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public Gyro getGyro()  {
        return gyro;
    }

    public DriveTrain getDriveTrain()  {
        return drivetrain;
    }

    public void initialize() {
        switch (Constants.getRobot()) {
            case ROBOT_2023:
            {
                drivetrain.init(this.hardwareMap, activeOpMode);
                lift.init(this.hardwareMap, activeOpMode);
                arm.init(this.hardwareMap, activeOpMode);
                gyro.init(this.hardwareMap);
                intake.init(this.hardwareMap, activeOpMode);
                break;
            }
            case ROBOT_CHASSIS:
            {
                drivetrain.init(this.hardwareMap, activeOpMode);
                break;
            }
            default:
                break;
        }
    }
}

