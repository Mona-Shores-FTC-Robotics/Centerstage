package com.example.meepmeeptesting;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;
import static com.example.meepmeeptesting.Constants.*;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public class Routes {
    private static DriveShim roadRunnerDrive = MeepMeepTesting.roadRunnerDrive;

    //Routes
    public static Action redAudienceBotTeamPropCenterRoute;
    public static Action redAudienceBotTeamPropLeftRoute;
    public static Action redAudienceBotTeamPropRightRoute;

    public static Action redBackstageBotTeamPropCenterRoute;
    public static Action redBackstageBotTeamPropLeftRoute;
    public static Action redBackstageBotTeamPropRightRoute;

    public static Action blueBackstageBotTeamPropCenterRoute;
    public static Action blueBackstageBotTeamPropLeftRoute;
    public static Action blueBackstageBotTeamPropRightRoute;

    public static Action blueAudienceBotTeamPropCenterRoute;
    public static Action blueAudienceBotTeamPropLeftRoute;
    public static Action blueAudienceBotTeamPropRightRoute;


    public static void BuildTeamPropCenterRoutes() {
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .setReversed(true)
                .splineToLinearHeading(BLUE_TRUSS_BACKSTAGE, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_TRUSS_FRONTSTAGE, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_WING, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
                .splineToLinearHeading(RED_SAFE_STRAFE, FACE_TOWARD_BLUE)
                .splineToLinearHeading(new Pose2d(RED_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_BLUE), FACE_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, FACE_TOWARD_RED)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .setReversed(true)
                .splineToLinearHeading(BLUE_TRUSS_BACKSTAGE, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_TRUSS_FRONTSTAGE, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_WING, FACE_TOWARD_FRONTSTAGE)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
//                .splineToLinearHeading(BLUE_STEAL_POSITION, STEAL_CENTER)
                .build();
    }

    public static void BuildTeamPropLeftRoutes() {
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, FACE_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();
    }

    public static void BuildTeamPropRightRoutes() {
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_45_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_45_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();

        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build();
    }
}
