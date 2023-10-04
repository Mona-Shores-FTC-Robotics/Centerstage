package com.example.meepmeeptesting;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_CENTERSPIKE;
import static com.example.meepmeeptesting.Constants.BLUE_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_EXIT;
import static com.example.meepmeeptesting.Constants.BLUE_THROUGH_DOOR;
import static com.example.meepmeeptesting.Constants.FACE_135_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_225_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_FRONTSTAGE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_RED;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_PARK;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_PIXEL_CENTERSPIKE;
import static com.example.meepmeeptesting.Constants.RED_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_EXIT;
import static com.example.meepmeeptesting.Constants.RED_THROUGH_DOOR;
import static com.example.meepmeeptesting.MeepMeepRobots.blueAudienceBot;
import static com.example.meepmeeptesting.MeepMeepRobots.blueAudienceBotLeft;
import static com.example.meepmeeptesting.MeepMeepRobots.blueAudienceBotRight;
import static com.example.meepmeeptesting.MeepMeepRobots.blueBackstageBot;
import static com.example.meepmeeptesting.MeepMeepRobots.blueBackstageBotLeft;
import static com.example.meepmeeptesting.MeepMeepRobots.blueBackstageBotRight;
import static com.example.meepmeeptesting.MeepMeepRobots.redAudienceBot;
import static com.example.meepmeeptesting.MeepMeepRobots.redAudienceBotLeft;
import static com.example.meepmeeptesting.MeepMeepRobots.redAudienceBotRight;
import static com.example.meepmeeptesting.MeepMeepRobots.redBackstageBot;
import static com.example.meepmeeptesting.MeepMeepRobots.redBackstageBotLeft;
import static com.example.meepmeeptesting.MeepMeepRobots.redBackstageBotRight;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class RoutesSpikePickup1BackdropPickup2BackdropPark {
    private static DriveShim roadRunnerDrive = MeepMeepTesting.roadRunnerDrive;

    //Center Route Variables
    private static Action redAudienceBotTeamPropCenterRoute;
    private static Action redBackstageBotTeamPropCenterRoute;
    private static Action blueBackstageBotTeamPropCenterRoute;
    private static Action blueAudienceBotTeamPropCenterRoute;

    //Left Route Variables
    private static Action redAudienceBotTeamPropLeftRoute;
    private static Action blueBackstageBotTeamPropLeftRoute;
    private static Action redBackstageBotTeamPropLeftRoute;
    private static Action blueAudienceBotTeamPropLeftRoute;

    //Right Route Variables
    private static Action redBackstageBotTeamPropRightRoute;
    private static Action redAudienceBotTeamPropRightRoute;
    private static Action blueBackstageBotTeamPropRightRoute;
    private static Action blueAudienceBotTeamPropRightRoute;

    public static void BuildTeamPropCenterRoutes() {
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
                .build();
    }

    public static void BuildTeamPropLeftRoutes() {
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_315_DEGREES)
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, FACE_315_DEGREES)
                .build();
    }

    public static void BuildTeamPropRightRoutes() {
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_225_DEGREES)
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_45_DEGREES)
                .build();

        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .build();
    }

    static void setTeamPropCenterRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
    }

    static void setTeamPropLeftRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropLeftRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropLeftRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropLeftRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropLeftRoute);
    }

    static void setTeamPropRightRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropRightRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropRightRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropRightRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropRightRoute);
    }

    static void setTeamPropAllRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
        blueBackstageBotLeft.runAction(blueBackstageBotTeamPropLeftRoute);
        blueBackstageBotRight.runAction(blueBackstageBotTeamPropRightRoute);

        blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
        blueAudienceBotLeft.runAction(blueAudienceBotTeamPropLeftRoute);
        blueAudienceBotRight.runAction(blueAudienceBotTeamPropRightRoute);

        redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
        redBackstageBotLeft.runAction(redBackstageBotTeamPropLeftRoute);
        redBackstageBotRight.runAction(redBackstageBotTeamPropRightRoute);

        redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
        redAudienceBotLeft.runAction(redAudienceBotTeamPropLeftRoute);
        redAudienceBotRight.runAction(redAudienceBotTeamPropRightRoute);
    }


}
