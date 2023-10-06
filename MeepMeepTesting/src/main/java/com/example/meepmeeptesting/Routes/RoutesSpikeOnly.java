package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.*;
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
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class RoutesSpikeOnly {
    private static DriveShim roadRunnerDrive = MeepMeepTesting.roadRunnerDrive;

    //Variables to store routes for team prop center for all four start locations
    private static Action redAudienceBotTeamPropCenterRoute;
    private static Action redBackstageBotTeamPropCenterRoute;
    private static Action blueBackstageBotTeamPropCenterRoute;
    private static Action blueAudienceBotTeamPropCenterRoute;

    //Variables to store routes for team prop left for all four start locations
    private static Action redBackstageBotTeamPropLeftRoute;
    private static Action blueAudienceBotTeamPropLeftRoute;
    private static Action redAudienceBotTeamPropLeftRoute;
    private static Action blueBackstageBotTeamPropLeftRoute;

    //Variables to store routes for team prop right for all four start locations
    private static Action redBackstageBotTeamPropRightRoute;
    private static Action redAudienceBotTeamPropRightRoute;
    private static Action blueBackstageBotTeamPropRightRoute;
    private static Action blueAudienceBotTeamPropRightRoute;

    public static void BuildRoutes() {
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_TOWARD_RED)
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, FACE_TOWARD_RED)
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_TOWARD_BLUE)
                .build();

        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_TOWARD_RED)
                .build();

        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_TOWARD_RED)
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_TOWARD_BLUE)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_TOWARD_BLUE)
                .build();

        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .build();
    }

    /**
     * METHODS TO SET SPIKE PIXEL ONLY ROUTES FOR ALL TEAM PROP LOCATIONS
     **/

    public static void setTeamPropCenterRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
    }

    public static void setTeamPropLeftRoutes() {
        blueBackstageBot.runAction(RoutesSpikeOnly.blueBackstageBotTeamPropLeftRoute);
        redBackstageBot.runAction(RoutesSpikeOnly.redBackstageBotTeamPropLeftRoute);
        redAudienceBot.runAction(RoutesSpikeOnly.redAudienceBotTeamPropLeftRoute);
        blueAudienceBot.runAction(RoutesSpikeOnly.blueAudienceBotTeamPropLeftRoute);
    }

    public static void setTeamPropRightRoutes() {
        blueBackstageBot.runAction(blueBackstageBotTeamPropRightRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropRightRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropRightRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropRightRoute);
    }

    public static void setTeamPropAllRoutes() {
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
