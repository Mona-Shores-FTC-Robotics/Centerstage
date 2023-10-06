package org.firstinspires.ftc.teamcode.ObjectClasses.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
import static org.firstinspires.ftc.teamcode.OpModes.Spike_Backdrop_Park_Auto.roadRunnerDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public class RoutesSpikeBackdropPark {

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
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, TANGENT_45_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, TANGENT_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_AUDIENCE_SPIKE_C), FACE_TOWARD_AUDIENCE), TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_AUDIENCE), TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_135_DEGREES)
                .build();

    }
}
