package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "Spike Only Auto")
public class Spike_Only_Auto extends LinearOpMode {

    private MecanumDriveMona mecanumDrive;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private  InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {
        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        /** Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);

        /** Initialize Gamepad and Robot - Order Important **/
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        
        SpikeOnlyRoute.BuildRoutes(mecanumDrive);

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

        //These should be set properly based on vision/override - team prop location cannot be overridden
        teamPropLoc = MatchConfig.finalTeamPropLocation;
        allianceColor = MatchConfig.finalAllianceColor;
        sideOfField = MatchConfig.finalSideOfField;

        //Set the starting pose of the robot
        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor , sideOfField);

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPose();


        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        //Check each AllianceColor/SideOfField combination and drive the route according to the team prop location
        CheckBlueBackstage();
        CheckBlueAudience();
        CheckRedBackstage();
        CheckRedAudience();

        Robot.getInstance().getGyroSubsystem().DriverStationTelemetry();
        Robot.getInstance().getActiveOpMode().telemetry.update();

        Actions.runBlocking(selectedRoute);
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getDriveSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getGyroSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getVisionSubsystem());
        Robot.reset();
    }

    private boolean CheckRedAudience() {
        if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            mecanumDrive.pose = RED_AUDIENCE_START_POSE;
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
            mecanumDrive.pose = RED_BACKSTAGE_START_POSE;
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
            mecanumDrive.pose = BLUE_AUDIENCE_START_POSE;
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
            mecanumDrive.pose = BLUE_BACKSTAGE_START_POSE;
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

