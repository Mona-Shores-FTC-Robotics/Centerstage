package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Routes.RoutesSpikeOnly.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Routes.RoutesSpikeOnly;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

@Autonomous(name = "Spike Only Auto")
public class Spike_Only_Auto extends LinearOpMode {

    Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);
    private DriveSubsystem roadRunnerDriveSubsystem;
//    public static MecanumDrive roadRunnerDrive;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {

        //Initialize the Robot
        robot.initialize();

        //initialize the Gamepads
        GamepadHandling.init();
        robot.getVisionSubsystem().SwitchToInitVisionProcessor();

//       roadRunnerDrive = new MecanumDrive(Robot.getInstance().getHardwareMap(), new Pose2d(0, 0, 0));
        roadRunnerDriveSubsystem = Robot.getInstance().getDriveSubsystem();
        RoutesSpikeOnly.BuildRoutes(roadRunnerDriveSubsystem);

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            robot.getVisionSubsystem().telemetryForInitProcessing();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Reset Gyro
        robot.getGyroSubsystem().resetAbsoluteYaw();

        //Display the initVision telemetry a final time
        robot.getVisionSubsystem().telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocationFinal();
        allianceColor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal();
        sideOfField = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getSideOfFieldFinal();

        //Set the starting pose of the robot
        robot.getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //After Init switch the vision processing to AprilTags
        robot.getVisionSubsystem().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        //Check each AllianceColor/SideOfField combination and drive the route according to the team prop location
        CheckBlueBackstage();
        CheckBlueAudience();
        CheckRedBackstage();
        CheckRedAudience();

        telemetry.clearAll();

        Actions.runBlocking(selectedRoute);

        sleep(30000);

    }

    private boolean CheckRedAudience() {
        if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            roadRunnerDriveSubsystem.pose = RED_AUDIENCE_START_POSE;
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
            roadRunnerDriveSubsystem.pose = RED_BACKSTAGE_START_POSE;
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
            roadRunnerDriveSubsystem.pose = BLUE_AUDIENCE_START_POSE;
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
            roadRunnerDriveSubsystem.pose = BLUE_BACKSTAGE_START_POSE;
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

            double x = roadRunnerDriveSubsystem.pose.position.x;
            double y = roadRunnerDriveSubsystem.pose.position.y;
            double combined = roadRunnerDriveSubsystem.pose.heading.real + roadRunnerDriveSubsystem.pose.heading.imag;

            telemetry.addData("Current Pose x", "%.1f", x);
            telemetry.addData("Current Pose y", "%.1f", y);
            telemetry.addData("Current Heading", "%.1f", combined);

            telemetryPacket.put("myX", x);

            telemetry.update();

            return false;
        }
    }
}

