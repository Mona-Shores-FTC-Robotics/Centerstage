package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name = "Basic_Auto")
public class Basic_Auto extends LinearOpMode {

    /** Create the robot **/
    Robot robot = Robot.createInstance(this);

    @Override
    public void runOpMode() {

        MecanumDrive roadRunnerDrive = new MecanumDrive(Robot.getInstance().getHardwareMap(), new Pose2d(0,0,0));
        //Set the type of Robot
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);

        //Initialize the Robot
        robot.initialize(robot.getHardwareMap());

        //initialize the Gamepads
        GamepadHandling.init();
        robot.getVision().SwitchToInitVisionProcessor();

        while (opModeInInit()) {
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            // Add Vision Init Processor Telemetry
            robot.getVision().getInitVisionProcessor().telemetryForInitProcessing();

            robot.getVision().getInitVisionProcessor().lockColorAndSide();

            telemetry.update();
            sleep(10);
        }

        //Display the initVision telemetry a final time
        robot.getVision().getInitVisionProcessor().telemetryForInitProcessing();
        telemetry.update();

        //After Init switch the vision processing to AprilTags
        robot.getVision().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        Actions.runBlocking(
                    roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        }
    }

