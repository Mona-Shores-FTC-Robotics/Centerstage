package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.Commands;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class Robot {

    private static Robot robot = null;
    public RobotType robotType;
    public OpModeType opModeType;

    public enum RobotType {
        ROBOT_CENTERSTAGE,
        ROBOT_DRIVE_BASE,
        ROBOT_VISION,
        ROBOT_SCORING_ARM,
        ROBOT_INTAKE
    }

    public enum OpModeType {
        TELEOP, AUTO
    }

    public Robot(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    private void initTele() {
        // initialize teleop-specific scheduler

        //Create Commands
        Commands.MakeTeleOpCommands();
    }

    private void initAuto() {
        // initialize auto-specific scheduler
    }


    private static ElapsedTime teleOpRuntime;
    private static LinearOpMode activeOpMode;
    private static DriveSubsystem mecanumDriveSubsystem;
    private static GyroSubsystem gyroSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static EndEffectorSubsystem endEffectorSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;

    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType rType, OpModeType oType) {
        activeOpMode = opMode;
        robotType = rType;
        opModeType = oType;
        HardwareMap hardwareMap = opMode.hardwareMap;
        teleOpRuntime = new ElapsedTime();

        CreateAndInitSubsystems(hardwareMap);
    }

    public void init(){
        //Initialize the Robot
        if (opModeType == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    private void CreateAndInitSubsystems(HardwareMap hardwareMap) {
        switch (robotType) {
            //Just the drive base
            case ROBOT_DRIVE_BASE: {
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                break;
            }

            case ROBOT_VISION: {
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                visionSubsystem.init();
                gyroSubsystem.init();
                mecanumDriveSubsystem.init();
                break;
            }

            case ROBOT_SCORING_ARM: {
                endEffectorSubsystem = new EndEffectorSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                endEffectorSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();
                break;
            }

            case ROBOT_INTAKE: {
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake");
                intakeSubsystem.init();
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

    public static Robot createInstance(LinearOpMode opMode, RobotType robotType, OpModeType opModeType ) {
        robot = new Robot(opMode, robotType, opModeType);
        return robot;
    }

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

    public GyroSubsystem getGyroSubsystem()  {return gyroSubsystem;    }
    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public EndEffectorSubsystem getEndEffectorSubsystem()  {return endEffectorSubsystem;}
    public LiftSlideSubsystem getLiftSlideSubsystem()  {return liftSlideSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}

}



