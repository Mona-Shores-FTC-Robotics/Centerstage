package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

public class RoutesSpikePickup1BackdropPickup2BackdropPark {
    static MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

    //Variables to store routes for team prop center for all four start locations
    public static Action redAudienceBotTeamPropCenterRoute;
    public static Action redBackstageBotTeamPropCenterRoute;
    public static Action blueBackstageBotTeamPropCenterRoute;
    public static Action blueAudienceBotTeamPropCenterRoute;

    //Variables to store routes for team prop left for all four start locations
    public static Action redBackstageBotTeamPropLeftRoute;
    public static Action blueAudienceBotTeamPropLeftRoute;
    public static Action redAudienceBotTeamPropLeftRoute;
    public static Action blueBackstageBotTeamPropLeftRoute;

    //Variables to store routes for team prop right for all four start locations
    public static Action redBackstageBotTeamPropRightRoute;
    public static Action redAudienceBotTeamPropRightRoute;
    public static Action blueBackstageBotTeamPropRightRoute;
    public static Action blueAudienceBotTeamPropRightRoute;

    public static void BuildRoutes() {
        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        blueBackstageBotTeamPropLeftRoute = mecanumDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        Action redBackstageBotTeamPropRightRoute1 = mecanumDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_115_DEGREES)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .setReversed(true)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, FACE_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        Action redBackstageBotTeamPropRightRoute2 = new AutoDriveToBackDrop();
        Action redBackstageBotTeamPropRightRoute9 = new AutoDriveToBackDrop();
//        Action redBackstageBotTeamPropRightRoute2a = mecanumDrive.actionBuilder(RED_BACKDROP_CENTER)
//                .build();

        Action redBackstageBotTeamPropRightRoute3 = mecanumDrive.actionBuilder(RED_BACKDROP_RIGHT)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_STAGING), TANGENT_TOWARD_AUDIENCE)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();


        Action redBackstageBotTeamPropRightRoute4 = mecanumDrive.actionBuilder(RED_BACKDROP_RIGHT)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_STAGING), TANGENT_TOWARD_AUDIENCE)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        Action redBackstageBotTeamPropRightRoute5 = mecanumDrive.actionBuilder(RED_BACKDROP_CENTER)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        redBackstageBotTeamPropRightRoute = new SequentialAction(
                redBackstageBotTeamPropRightRoute1,
                redBackstageBotTeamPropRightRoute2,
//                redBackstageBotTeamPropRightRoute2a,
                redBackstageBotTeamPropRightRoute3,
                redBackstageBotTeamPropRightRoute9,
                redBackstageBotTeamPropRightRoute4,
                redBackstageBotTeamPropRightRoute5
        );


        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = mecanumDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropLeftRoute = mecanumDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = mecanumDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, FACE_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropCenterRoute = mecanumDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = mecanumDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = mecanumDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_45_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute = mecanumDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_AUDIENCE_SPIKE_C), FACE_TOWARD_AUDIENCE), TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropLeftRoute = mecanumDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_AUDIENCE), TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = mecanumDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropCenterRoute = mecanumDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

    }

}
