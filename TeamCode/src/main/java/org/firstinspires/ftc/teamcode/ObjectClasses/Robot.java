package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Controllers.DriveController;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArm;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;

public class Robot {

    private static Robot robot = null;
    public static RobotType robotType;

    public enum RobotType {
        ROBOT_CENTERSTAGE,
        ROBOT_DRIVE_BASE,
        ROBOT_VISION,
        ROBOT_SCORING_ARM,
        ROBOT_INTAKE
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

    private static ScoringArm scoringArm;
    private static DriveController driveController;

    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType type) {
        activeOpMode = opMode;
        robotType = type;
        HardwareMap hardwareMap = opMode.hardwareMap;
        teleOpRuntime = new ElapsedTime();

        switch (robotType) {
            //Just the drive base
            case ROBOT_DRIVE_BASE: {
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, "LFDrive", "LBDrive", "RFDrive", "RBDRive");
                driveController = new DriveController();
                break;
            }

            case ROBOT_VISION: {
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, "LFDrive", "LBDrive", "RFDrive", "RBDRive");
                driveController = new DriveController();
                break;
            }

            case ROBOT_SCORING_ARM: {
                endEffectorSubsystem = new EndEffectorSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                scoringArm = new ScoringArm();
                break;
            }

            case ROBOT_INTAKE: {
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake");
                break;
            }

            case ROBOT_CENTERSTAGE: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap, "LFDrive", "LBDrive", "RFDrive", "RBDRive");
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake");
                endEffectorSubsystem = new EndEffectorSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                driveController = new DriveController();
                scoringArm = new ScoringArm();

                //airplane launcher
                //winch
                //intake pick up
                //lights
                break;
            }
        }
    }

    public static Robot createInstance(LinearOpMode opMode, RobotType type) {
        robot = new Robot(opMode, type);
        return robot;
    }

    public void initialize() {
       if (visionSubsystem!=null) visionSubsystem.init();
       if (gyroSubsystem!=null) gyroSubsystem.init();
       if (mecanumDriveSubsystem!=null) mecanumDriveSubsystem.init();
       if (intakeSubsystem!=null) intakeSubsystem.init();
       if (endEffectorSubsystem!=null) endEffectorSubsystem.init();
       if (liftSlideSubsystem!=null) liftSlideSubsystem.init();
       if (shoulderSubsystem!=null) shoulderSubsystem.init();
       if (scoringArm!=null) scoringArm.init();
       if (driveController!=null) driveController.init();
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

    public GyroSubsystem getGyroSubsystem()  {return gyroSubsystem;    }
    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public EndEffectorSubsystem getEndEffectorSubsystem()  {return endEffectorSubsystem;}
    public LiftSlideSubsystem getLiftSlideSubsystem()  {return liftSlideSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public DriveController getDriveController()  {return driveController;}
    public ScoringArm getScoringArm()  {return scoringArm;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}

}



