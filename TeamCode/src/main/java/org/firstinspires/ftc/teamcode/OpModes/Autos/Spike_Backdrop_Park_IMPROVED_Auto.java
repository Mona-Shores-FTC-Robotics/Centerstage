package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.blueBackstageBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.redBackstageBotTeamPropRightRoute;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropParkImproved;

@Autonomous(name = "Spike Backdrop Park Auto IMPROVED")
public class Spike_Backdrop_Park_IMPROVED_Auto extends LinearOpMode {

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

        // Create and Initialize the robot
        Robot.createInstance(this, Robot.RobotType.ROBOT_CENTERSTAGE);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO, gamepadHandling);

        // Turn on the Init Vision Processor to Automatically Figure Out Alliance Color, Side, and Team Prop Location
        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        //Build all the routes so we can select one quickly later
        RoutesSpikeBackdropParkImproved.BuildRoutes();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing();

            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = MatchConfig.finalTeamPropLocation;
        allianceColor = MatchConfig.finalAllianceColor;
        sideOfField = MatchConfig.finalSideOfField;

        //Set the starting pose based on the Alliance Color and Side of the Field
        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //Reset Gyro to Zero and set the offset to the current heading of the Robot pose
        //For example:
        //  if we are Backstage Red, the relativeYaw is set to 90 and the gyro is set to 0 and the offset is set to 90
        //  if we are Audience Blue, the relativeYaw is set to -90 and the gyro is set to 0 and the offset is set to -90
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //this saves the alliance color in a spot that persists between opModes
        MatchConfig.finalAllianceColor = allianceColor;

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        //Check each AllianceColor/SideOfField combination and drive the route according to the team prop location
        CheckBlueBackstage();
        CheckBlueAudience();
        CheckRedBackstage();
        CheckRedAudience();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRoute);
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousRelativeYawDegrees = Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees;
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getGyroSubsystem().offsetFromAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().mecanumDrive.pose;
    }

    private boolean CheckRedAudience() {
        if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = RED_AUDIENCE_START_POSE;
            if (teamPropLoc == InitVisionProcessor.TeamPropLocation.LEFT) {
                selectedRoute = redAudienceBotTeamPropLeftRoute;
            } else if (teamPropLoc == InitVisionProcessor.TeamPropLocation.RIGHT) {
                selectedRoute = redAudienceBotTeamPropRightRoute;
            } else {
                selectedRoute = redAudienceBotTeamPropCenterRoute;
            }
            return true;
        }
        return false;
    }

    private boolean CheckRedBackstage() {
        if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE) {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = RED_BACKSTAGE_START_POSE;
            if (teamPropLoc == InitVisionProcessor.TeamPropLocation.LEFT) {
                selectedRoute = redBackstageBotTeamPropLeftRoute;
            } else if (teamPropLoc == InitVisionProcessor.TeamPropLocation.RIGHT) {
                selectedRoute = redBackstageBotTeamPropRightRoute;
            } else {
                selectedRoute = redBackstageBotTeamPropCenterRoute;
            }
            return true;
        }
        return false;
    }

    private boolean CheckBlueAudience() {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = BLUE_AUDIENCE_START_POSE;
            if (teamPropLoc == InitVisionProcessor.TeamPropLocation.LEFT) {
                selectedRoute = blueAudienceBotTeamPropLeftRoute;
            } else if (teamPropLoc == InitVisionProcessor.TeamPropLocation.RIGHT) {
                selectedRoute = blueAudienceBotTeamPropRightRoute;
            } else {
                selectedRoute = blueAudienceBotTeamPropCenterRoute;
            }
            return true;
        }
        return false;
    }

    private boolean CheckBlueBackstage() {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE) {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = BLUE_BACKSTAGE_START_POSE;
            if (teamPropLoc == InitVisionProcessor.TeamPropLocation.LEFT) {
                selectedRoute = blueBackstageBotTeamPropLeftRoute;
            } else if (teamPropLoc == InitVisionProcessor.TeamPropLocation.RIGHT) {
                selectedRoute = blueBackstageBotTeamPropRightRoute;
            } else {
                selectedRoute = blueBackstageBotTeamPropCenterRoute;
            }
            return true;
        }
        return false;
    }
}

