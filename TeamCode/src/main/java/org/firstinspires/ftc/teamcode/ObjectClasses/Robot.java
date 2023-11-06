package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class Robot {

    private static Robot robot = null;
    public RobotType robotType;
    public enum RobotType {ROBOT_CENTERSTAGE, ROBOT_DRIVE_BASE, ROBOT_VISION, ROBOT_SCORING_ARM, ROBOT_INTAKE}
    public enum OpModeType {TELEOP, AUTO}

    private static LinearOpMode activeOpMode;
    private static DriveSubsystem mecanumDriveSubsystem;
    private static GyroSubsystem gyroSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static EndEffectorSubsystem endEffectorSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;

    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType rType) {
        activeOpMode = opMode;
        robotType = rType;
        HardwareMap hardwareMap = opMode.hardwareMap;
        CreateSubsystems(hardwareMap);
    }

    private void CreateSubsystems(HardwareMap hardwareMap) {
        switch (robotType) {
            //Just the drive base
            case ROBOT_DRIVE_BASE: {
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                break;
            }

            case ROBOT_VISION: {
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                break;
            }

            case ROBOT_SCORING_ARM: {
//                endEffectorSubsystem = new EndEffectorSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                break;
            }

            case ROBOT_INTAKE: {
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake");
                break;
            }

            case ROBOT_CENTERSTAGE: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake");
                endEffectorSubsystem = new EndEffectorSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");

                //airplane launcher
                //winch
                //intake pick up
                //lights
                break;
            }
        }
    }

    //Ensures only one robot object is ever created
    public static Robot createInstance(LinearOpMode opMode, RobotType robotType) {

        robot = new Robot(opMode, robotType);
        return robot;
    }

    public static void reset() {
        robot = null;
    }


    // Static method to get single instance of Robot
    public static synchronized Robot getInstance() {
        if (robot == null) {
            telemetry.addLine("error");
        }
        return robot;
    }


    // Initialize teleop or autonomous, depending on which is used
    public void init(OpModeType opModeType){
        if (opModeType == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    private void initTele() {
        switch (robotType) {
            case ROBOT_DRIVE_BASE: {
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                break;
            }
            case ROBOT_VISION: {
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                visionSubsystem.init();
                break;
            }
            case ROBOT_SCORING_ARM: {
//                endEffectorSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();
                RobotCommands.MakeRobotScoringArmCommands();
                break;
            }
            case ROBOT_INTAKE: {
                intakeSubsystem.init();
                break;
            }
            case ROBOT_CENTERSTAGE: {
                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                intakeSubsystem.init();
                endEffectorSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();
                //Systems to be added:
                //Servo - Drone launch release

                //Motor - Winch for hanging
                //Lights for identifying pixels in intake
                break;
            }
        }
    }

    private void initAuto() {
        // initialize auto-specific scheduler

        switch (robotType) {

            case ROBOT_VISION: {
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                visionSubsystem.init();
                break;
            }

            case ROBOT_CENTERSTAGE: {
                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                intakeSubsystem.init();
                endEffectorSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();

                //airplane launcher
                //winch
                //intake pick up
                //lights
                break;
            }
        }
    }

    public GyroSubsystem getGyroSubsystem()  {return gyroSubsystem;    }
    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public EndEffectorSubsystem getEndEffectorSubsystem()  {return endEffectorSubsystem;}
    public LiftSlideSubsystem getLiftSlideSubsystem()  {return liftSlideSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}
}



