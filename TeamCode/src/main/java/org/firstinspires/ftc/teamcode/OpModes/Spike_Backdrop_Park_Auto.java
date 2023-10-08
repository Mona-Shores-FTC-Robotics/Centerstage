package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Routes.RoutesSpikeBackdropPark.*;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Routes.RoutesSpikeBackdropPark;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Autonomous(name = "Spike Backdrop Park Auto")
public class Spike_Backdrop_Park_Auto extends LinearOpMode {

    Robot robot = Robot.createInstance(this);

    public static MecanumDrive roadRunnerDrive;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {

        //Set the type of Robot
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);

        //Initialize the Robot
        robot.initialize(robot.getHardwareMap());

        //initialize the Gamepads
        GamepadHandling.init();
        robot.getVision().SwitchToInitVisionProcessor();

        roadRunnerDrive = new MecanumDrive(Robot.getInstance().getHardwareMap(), new Pose2d(0, 0, 0));

        RoutesSpikeBackdropPark.BuildRoutes(roadRunnerDrive);

        while (opModeInInit()) {
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            // Add Vision Init Processor Telemetry
            robot.getVision().telemetryForInitProcessing();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }


        //Reset Gyro
        robot.getGyro().resetAbsoluteYaw();

        //Display the initVision telemetry a final time
        robot.getVision().telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = Robot.getInstance().getVision().getInitVisionProcessor().getTeamPropLocationFinal();
        allianceColor = Robot.getInstance().getVision().getInitVisionProcessor().getAllianceColorFinal();
        sideOfField = Robot.getInstance().getVision().getInitVisionProcessor().getSideOfFieldFinal();

        //After Init switch the vision processing to AprilTags
        robot.getVision().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        //Check each AllianceColor/SideOfField combination and drive the route according to the team prop location
        CheckBlueBackstage();
        CheckBlueAudience();
        CheckRedBackstage();
        CheckRedAudience();

        telemetry.clearAll();
        telemetry.setAutoClear(false);

        Actions.runBlocking(selectedRoute);

        sleep(30000);

    }

    private boolean CheckRedAudience() {
        if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            roadRunnerDrive.pose = RED_AUDIENCE_START_POSE;
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
            roadRunnerDrive.pose = RED_BACKSTAGE_START_POSE;
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
            roadRunnerDrive.pose = BLUE_AUDIENCE_START_POSE;
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
            roadRunnerDrive.pose = BLUE_BACKSTAGE_START_POSE;
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

    public class PoseTelemetry implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double x = roadRunnerDrive.pose.position.x;
            double y = roadRunnerDrive.pose.position.y;
            double combined = roadRunnerDrive.pose.heading.real + roadRunnerDrive.pose.heading.imag;

            telemetry.addData("Current Pose x", "%.1f", x);
            telemetry.addData("Current Pose y", "%.1f", y);
            telemetry.addData("Current Heading", "%.1f", combined);

            telemetryPacket.put("myX", x);

            telemetry.update();

            return false;
        }
    }
}

