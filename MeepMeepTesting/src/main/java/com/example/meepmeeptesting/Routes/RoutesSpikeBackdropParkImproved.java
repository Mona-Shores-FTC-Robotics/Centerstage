package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_LANE_A;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.BLUE_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.BLUE_THROUGH_DOOR;
import static com.example.meepmeeptesting.Constants.FACE_135_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_225_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R_PAST;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_LANE_F;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.RED_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.RED_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.RED_THROUGH_DOOR;
import static com.example.meepmeeptesting.Constants.TANGENT_135_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_225_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_315_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_45_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_75_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_RED;
import static com.example.meepmeeptesting.Constants.TILE;
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
import static com.example.meepmeeptesting.Routes.RoutesSpikeBackdropParkImproved.LiftStates.AUTO_HIGH;
import static com.example.meepmeeptesting.Routes.RoutesSpikeBackdropParkImproved.LiftStates.AUTO_LOW;
import static com.example.meepmeeptesting.Routes.RoutesSpikeBackdropParkImproved.LiftStates.AUTO_MID;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class RoutesSpikeBackdropParkImproved {
    private static DriveShim roadRunnerDrive = MeepMeepTesting.roadRunnerDrive;

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

    public enum LiftStates {
        AUTO_LOW, AUTO_MID, AUTO_HIGH, MAX, HIGH, MID, LOW, SAFE, HOME, ZERO, MANUAL;
    }

    public static void BuildRoutes() {

        Action dropPurple = new SleepAction(.1);

        //////////
        // LEFT //
        //////////

        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        ///////////
        // RIGHT //
        ///////////

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R_PAST, Math.toRadians(55))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R_DROP, Math.toRadians(55))
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();


        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_L), TANGENT_TOWARD_RED)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();

        ////////////
        // CENTER //
        ////////////

        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();
    }

    public static class ActionsForSpikeBackdrop {
        public Action ScoreAndBackup(Pose2d start_pose, LiftStates liftHeight) {
            return roadRunnerDrive.actionBuilder(start_pose)
                    .lineToX(TILE * 2 + 7)
                    .lineToX(TILE * 2 - 5.5)
                    .build();
        }

        public Action ScoreWithTwoHeightsAndBackup(Pose2d start_pose, LiftStates firstHeight, LiftStates secondHeight) {
            return roadRunnerDrive.actionBuilder(start_pose)
                    .lineToX(TILE * 2 + 7)
                    .lineToX(TILE * 2 - 2)
                    .build();

        }
    }

        /**
         * METHODS TO SET SIMPLE ROUTES FOR ALL TEAM PROP LOCATIONS
         **/

        public static void setTeamPropCenterRoutes() {
            blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
            blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
            redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
            redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
        }

        public static void setTeamPropLeftRoutes() {
            blueBackstageBot.runAction(blueBackstageBotTeamPropLeftRoute);
            blueAudienceBot.runAction(blueAudienceBotTeamPropLeftRoute);
            redBackstageBot.runAction(redBackstageBotTeamPropLeftRoute);
            redAudienceBot.runAction(redAudienceBotTeamPropLeftRoute);
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
