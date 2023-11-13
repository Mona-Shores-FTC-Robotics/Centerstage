package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKDROP_STAGING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_NEUTRAL_PIXEL_WING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.blueBackstageBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redAudienceBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redAudienceBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redAudienceBotTeamPropRightRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redBackstageBotTeamPropCenterRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redBackstageBotTeamPropLeftRoute;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute.redBackstageBotTeamPropRightRoute;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.SpikeOnlyRoute;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "TEST - Forward + StopAndAdd(AutoDriveAction) + Backward")
public class Test_ForwardRoute_AutoDriveToBackdropAction_BackwardRoute extends LinearOpMode {
    private MecanumDriveMona roadRunnerDriveSubsystem;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private  InitVisionProcessor.AllianceColor allianceColor;
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
        SpikeOnlyRoute.BuildRoutes(roadRunnerDriveSubsystem);

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

        teamPropLoc = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocation();
        allianceColor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor;
        sideOfField = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().sideOfField;

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

        Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE);

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPose();


        Action testRoute = Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE))
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_BACKDROP_STAGING), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new AutoDriveToBackDrop())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_BACKSTAGE_SPIKE_L), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_AUDIENCE)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_NEUTRAL_PIXEL_WING), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_AUDIENCE)
                .build();

        //how should roadrunner handle changes in telemetry?
        Actions.runBlocking(testRoute);

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

