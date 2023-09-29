package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Autonomous(name = "Basic_Auto")
public class Basic_Auto extends LinearOpMode {

    /** Create the robot **/
    Robot robot = Robot.createInstance(this);

    //Routes
    public Action redAudienceBotTeamPropCenterRoute;
    public Action redAudienceBotTeamPropLeftRoute;
    public Action redAudienceBotTeamPropRightRoute;

    public Action redBackstageBotTeamPropCenterRoute;
    public Action redBackstageBotTeamPropLeftRoute;
    public Action redBackstageBotTeamPropRightRoute;

    public Action blueBackstageBotTeamPropCenterRoute;
    public Action blueBackstageBotTeamPropLeftRoute;
    public Action blueBackstageBotTeamPropRightRoute;

    public Action blueAudienceBotTeamPropCenterRoute;
    public Action blueAudienceBotTeamPropLeftRoute;
    public Action blueAudienceBotTeamPropRightRoute;

    @Override
    public void runOpMode() {

        MecanumDrive roadRunnerDrive = new MecanumDrive(Robot.getInstance().getHardwareMap(), BLUE_RIGHT_START_POSE);
        //Set the type of Robot
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);

        //Initialize the Robot
        robot.initialize(robot.getHardwareMap());

        //initialize the Gamepads
        GamepadHandling.init();
        robot.getVision().SwitchToInitVisionProcessor();

        //BuildTeamPropCenterRoutes(roadRunnerDrive);
        BuildTeamPropLeftRoutes(roadRunnerDrive);
        //BuildTeamPropRightRoutes(roadRunnerDrive);


        while (opModeInInit()) {
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            // Add Vision Init Processor Telemetry
            robot.getVision().telemetryForInitProcessing();

            robot.getVision().lockColorAndSide();

            telemetry.update();
            sleep(10);
        }

        //Display the initVision telemetry a final time
        robot.getVision().telemetryForInitProcessing();
        telemetry.update();

        //After Init switch the vision processing to AprilTags
        robot.getVision().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        //Reset Gyro
        robot.getGyro().resetYaw();


        Actions.runBlocking( blueAudienceBotTeamPropCenterRoute );

        }

    private void BuildTeamPropCenterRoutes(MecanumDrive roadRunnerDrive) {

        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
                .splineToLinearHeading(RED_SAFE_STRAFE , FACE_TOWARD_BLUE)
                .splineToLinearHeading(new Pose2d(RED_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_BLUE) , FACE_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
                .splineToLinearHeading(BLUE_SAFE_STRAFE , FACE_TOWARD_RED)
                .splineToLinearHeading(new Pose2d(BLUE_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_RED), FACE_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();
    }

    private void BuildTeamPropLeftRoutes(MecanumDrive roadRunnerDrive) {
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_RIGHT_START_POSE)
              .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .lineToX(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, FACE_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .lineToX(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();
    }

    private void BuildTeamPropRightRoutes(MecanumDrive roadRunnerDrive) {
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .lineToX(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(new Pose2d(RED_AUDIENCE_SPIKE_R.position.x, RED_AUDIENCE_SPIKE_R.position.y, FACE_135_DEGREES), FACE_TOWARD_BACKSTAGE)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RED_STAGEDOOR_ENTRANCE.position.x, RED_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToX(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(new Pose2d(BLUE_AUDIENCE_SPIKE_R.position.x, BLUE_AUDIENCE_SPIKE_R.position.y, FACE_315_DEGREES), FACE_315_DEGREES)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(BLUE_STAGEDOOR_ENTRANCE.position.x, BLUE_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_RED)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToX(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();
    }
}

