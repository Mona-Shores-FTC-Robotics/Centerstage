package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BLUE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.blueBackstageBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly.redBackstageBotTeamPropRightRoute;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBlueBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "Alignment Test Auto")
public class Alightment_Test_Auto extends LinearOpMode {

    private MecanumDriveMona roadRunnerDriveSubsystem;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {
        /** Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);

        /** Initialize Gamepad and Robot - Order Important **/
        GamepadHandling.init();
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        roadRunnerDriveSubsystem = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        Action testAutoAlignAction = new AutoDriveToBlueBackDrop();


        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing();
            GamepadHandling.getDriverGamepad().readButtons();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().resetAbsoluteYaw();

        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = MatchConfig.finalTeamPropLocation;
        allianceColor = MatchConfig.finalAllianceColor;
        sideOfField = MatchConfig.finalSideOfField;

        //Set the starting pose of the robot
        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //this saves the alliance color in a spot that persists between opModes
        MatchConfig.finalAllianceColor = allianceColor;

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        telemetry.clearAll();

        Actions.runBlocking(testAutoAlignAction);
    }
}

