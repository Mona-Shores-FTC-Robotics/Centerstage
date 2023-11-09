package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKDROP_STAGING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR;
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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes.RoutesSpikeOnly;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Utility.TelemetryAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Utility.TelemetryMona;

import java.util.Arrays;
@Config
@Autonomous(name = "Test Stop And Add")
public class Test_Auto extends LinearOpMode {
    static double forward_distance=35;
    private MecanumDriveMona roadRunnerDriveSubsystem;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private  InitVisionProcessor.AllianceColor allianceColor;
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
        RoutesSpikeOnly.BuildRoutes(roadRunnerDriveSubsystem);

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing();
            GamepadHandling.getDriverGamepad().readButtons();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }


        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocation();
        allianceColor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor;
        sideOfField = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().sideOfField;

        //Set the starting pose of the robot
//        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

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

//        ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(-20,20);
//        MinVelConstraint minVelConstraint = new MinVelConstraint(Arrays.asList(
//                Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(20),
//                new AngularVelConstraint(MotorParametersRR.maxAngVel)
//                ));

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().resetAbsoluteYaw();

        Robot.getInstance().getGyroSubsystem().setRelativeYawTo0();

        Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE);

        Action testRoute = Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE))
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_BACKDROP_STAGING), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_AUDIENCE)
                .stopAndAdd(new AutoDriveToBackDrop())
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_AUDIENCE)
                .build();


        //how should roadrunner handle changes in telemetry?

        Actions.runBlocking(testRoute);

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getDriveSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getGyroSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getVisionSubsystem());
        Robot.reset();
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
}

