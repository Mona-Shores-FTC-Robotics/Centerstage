package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "TEST - Just AutoDriveToBackdrop")
public class Test_AutoDriveToBackdropAction extends LinearOpMode {

    public static int DISTANCE_TO_TAG=18;
    private static VisionSubsystem.DeliverLocation DELIVER_LOCATION= VisionSubsystem.DeliverLocation.LEFT;

    private MecanumDriveMona roadRunnerDriveSubsystem;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        /** Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_CENTERSTAGE);

        /** Initialize Gamepad and Robot - Order Important **/
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        roadRunnerDriveSubsystem = Robot.getInstance().getDriveSubsystem().mecanumDrive;

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

        teamPropLoc = MatchConfig.finalTeamPropLocation;
        allianceColor = MatchConfig.finalAllianceColor;
        sideOfField = MatchConfig.finalSideOfField;

        //Set the starting pose of the robot
//        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //this saves the alliance color in a spot that persists between opModes
        MatchConfig.finalAllianceColor = allianceColor;

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        while(!Robot.getInstance().getVisionSubsystem().getVisionPortal().getProcessorEnabled(
                Robot.getInstance().getVisionSubsystem().getAprilTagProcessor()))
        {
            //wait for april tag processor to be enabled
        };

        telemetry.clearAll();

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();
        Actions.runBlocking( new AutoDriveToBackDrop(DELIVER_LOCATION));

//        Actions.runBlocking(new TurnToAction(0));

    }

}

