package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_STAGING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueBackstageBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.blueTestRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.redAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.redAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.redBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.redBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper.redBackstageBotTeamPropRightRoute;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSuper;

@Config
@Autonomous(name = "Pickup Test Auto")
public class Pickup_Test_Auto extends LinearOpMode {

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action testRoute;



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

        MatchConfig.CheckRobotConfig(hardwareMap);

        //Build all the routes so we can select one quickly later
        RoutesSuper.BuildRoutes();

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

        //Test Conditions
        Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = BLUE_NEUTRAL_STAGING;
        allianceColor = MatchConfig.finalAllianceColor = InitVisionProcessor.AllianceColor.BLUE;
        sideOfField = MatchConfig.finalSideOfField = InitVisionProcessor.SideOfField.AUDIENCE;
        teamPropLoc = MatchConfig.finalTeamPropLocation = InitVisionProcessor.TeamPropLocation.CENTER;

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        testRoute=blueTestRoute;

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();


        Actions.runBlocking(testRoute);

        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousRelativeYawDegrees = Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees;
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getGyroSubsystem().offsetFromAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().mecanumDrive.pose;

    }
}

