package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "TEST - Just AutoDriveToBackdrop")
public class Test_AutoDriveToBackdropAction extends LinearOpMode {

    private static final VisionSubsystem.DeliverLocation DELIVER_LOCATION= VisionSubsystem.DeliverLocation.LEFT;

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        // Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_CENTERSTAGE);

        // Initialize Gamepad and Robot
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing(gamepadHandling);
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing(gamepadHandling);
        telemetry.update();

        //Set the starting pose of the robot
//        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        while(!Robot.getInstance().getVisionSubsystem().getVisionPortal().getProcessorEnabled(
                Robot.getInstance().getVisionSubsystem().getAprilTagProcessor()))
        {
            //wait for april tag processor to be enabled
        }

        telemetry.clearAll();

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //Start the Auto Timer
        MatchConfig.OpModeTimer = new ElapsedTime();
        MatchConfig.OpModeTimer.reset();

        //Start the Loop Timer
        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();

        //Start the April Tag timestamp timer
        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking( new AutoDriveToBackDrop(DELIVER_LOCATION));

    }

}

