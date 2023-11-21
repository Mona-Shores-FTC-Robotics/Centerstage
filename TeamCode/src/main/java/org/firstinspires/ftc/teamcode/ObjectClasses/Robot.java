package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class Robot {

    private static Robot robot = null;
    public RobotType robotType;
    public OpModeType opModeType;
    public enum RobotType {ROBOT_CENTERSTAGE, ROBOT_DRIVE_BASE, ROBOT_VISION, ROBOT_SCORING_ARM, ROBOT_INTAKE, ROBOT_PIT_MODE}
    public enum OpModeType {TELEOP, AUTO}

    private static LinearOpMode activeOpMode;
    private static DriveSubsystem mecanumDriveSubsystem;
    private static GyroSubsystem gyroSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static GripperSubsystem gripperSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static DroneSubsystem droneSubsystem;

    private static ClimberSubsystem climberSubsystem;

    private static boolean hasInit = false;

    /* Constructor */
    private Robot(LinearOpMode opMode, RobotType rType) {
        activeOpMode = opMode;
        robotType = rType;
        HardwareMap hMap = opMode.hardwareMap;
        CreateSubsystems(hMap);
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
                gripperSubsystem = new GripperSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                droneSubsystem = new DroneSubsystem(hardwareMap, "drone");
                climberSubsystem = new ClimberSubsystem(hardwareMap, "climb", "climbWinch");
                break;
            }

            case ROBOT_INTAKE: {
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake", "intake2");
                break;
            }

            case ROBOT_CENTERSTAGE: {
                mecanumDriveSubsystem = new DriveSubsystem(hardwareMap);
                gyroSubsystem = new GyroSubsystem(hardwareMap, "imu");
                visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1");
                intakeSubsystem = new IntakeSubsystem(hardwareMap, "intake", "intake2");
                gripperSubsystem = new GripperSubsystem(hardwareMap, "endeffector");
                liftSlideSubsystem = new LiftSlideSubsystem(hardwareMap, "liftslide");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                droneSubsystem = new DroneSubsystem(hardwareMap, "drone");
                climberSubsystem = new ClimberSubsystem(hardwareMap, "climb", "climbWinch");
                break;
            }
            case ROBOT_PIT_MODE: {

                droneSubsystem = new DroneSubsystem(hardwareMap, "drone");
                climberSubsystem = new ClimberSubsystem(hardwareMap, "climb", "climbWinch");
                shoulderSubsystem = new ShoulderSubsystem(hardwareMap, "shoulder");
                gripperSubsystem = new GripperSubsystem(hardwareMap, "endeffector");
                break;
            }
        }
    }

    //Ensures only one robot object is ever created
    public static Robot createInstance(LinearOpMode opMode, RobotType robotType) {
        //there should only ever be one instance of robot - when we run a new opMode it should delete any previously existing robot object and make a new one
        //MatchConfig can house any data we need between opModes.
        robot=null;
        robot = new Robot(opMode, robotType);
        return robot;
    }

    public synchronized void reset() {
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
    public void init(OpModeType oType){
        opModeType = oType;
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
                gripperSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();
                droneSubsystem.init();
                climberSubsystem.init();
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
                gripperSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.initTele();
                droneSubsystem.init();
                climberSubsystem.init();
                 //Lights for identifying pixels in intake
                break;
            }
            case ROBOT_PIT_MODE: {
                droneSubsystem.init();
                climberSubsystem.init();
                gripperSubsystem.init();
                shoulderSubsystem.init();
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
                gripperSubsystem.init();
                liftSlideSubsystem.init();
                shoulderSubsystem.init();

                //todo do any subsystems need to be init during auto?

                break;
            }
        }
    }

    public GyroSubsystem getGyroSubsystem()  {return gyroSubsystem;    }
    public DriveSubsystem getDriveSubsystem()  {return mecanumDriveSubsystem;}
    public VisionSubsystem getVisionSubsystem()  {return visionSubsystem;}
    public IntakeSubsystem getIntakeSubsystem()  {return intakeSubsystem;}
    public GripperSubsystem getEndEffectorSubsystem()  {return gripperSubsystem;}
    public LiftSlideSubsystem getLiftSlideSubsystem()  {return liftSlideSubsystem;}
    public ShoulderSubsystem getShoulderSubsystem()  {return shoulderSubsystem;}
    public LinearOpMode getActiveOpMode()  {return activeOpMode;}
    public DroneSubsystem getDroneSubsystem(){return droneSubsystem;}

    public ClimberSubsystem getClimberSubsystem(){return climberSubsystem;};
}



