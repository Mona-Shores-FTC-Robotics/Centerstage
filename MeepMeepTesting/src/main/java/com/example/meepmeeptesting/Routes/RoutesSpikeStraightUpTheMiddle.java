package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_ALIGNMENT;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_ALIGNMENT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_PARK_LANE_A;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_PARK_LANE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_LANE_A;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_CENTERSPIKE;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_WING;
import static com.example.meepmeeptesting.Constants.BLUE_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.BLUE_THROUGH_DOOR;
import static com.example.meepmeeptesting.Constants.FACE_115_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_135_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_160_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_225_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_30_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_340_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_RED;
import static com.example.meepmeeptesting.Constants.HALF_TILE;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_PARK_LANE_D;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_PARK_LANE_F;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_LANE_F;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_PIXEL_WING;
import static com.example.meepmeeptesting.Constants.RED_SAFE_STRAFE;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.RED_THROUGH_DOOR;
import static com.example.meepmeeptesting.Constants.TANGENT_135_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_225_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_315_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_45_DEGREES;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_RED;
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
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class RoutesSpikeStraightUpTheMiddle {
    private static DriveShim roadRunnerDrive = MeepMeepTesting.roadRunnerDrive;

    //Variables to store routes for team prop center for all four start locations
    private static Action redAudienceBotTeamPropCenterRoute;
    private static Action redBackstageBotTeamPropCenterRoute;
    private static Action blueBackstageBotTeamPropCenterRoute;
    private static Action blueAudienceBotTeamPropCenterRoute;

    //Variables to store routes for team prop left for all four start locations
    private static Action redBackstageBotTeamPropLeftRoute;
    private static Action blueAudienceBotTeamPropLeftRoute;
    private static SequentialAction redAudienceBotTeamPropLeftRoute;
    private static Action blueBackstageBotTeamPropLeftRoute;

    //Variables to store routes for team prop right for all four start locations
    private static Action redBackstageBotTeamPropRightRoute;
    private static Action redAudienceBotTeamPropRightRoute;
    private static Action blueBackstageBotTeamPropRightRoute;
    private static Action blueAudienceBotTeamPropRightRoute;

    public static void BuildRoutes() {
        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        Action blueBackstageBotTeamPropLeftRoute1 = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_115_DEGREES)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .setReversed(true)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_WING, FACE_TOWARD_BACKSTAGE)
                .lineToX(BLUE_BACKDROP_CENTER.position.x)
                .build();

        CustomActions blah5 = new CustomActions();
        CustomActions blah6 = new CustomActions();
        Action blueBackstageBotTeamPropLeftRoute2 = blah5.alignToLeftSideOfBackDropWithAprilTag();

        Action blueBackstageBotTeamPropLeftRoute2a = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_CENTER)
                .build();

        Action blueBackstageBotTeamPropLeftRoute3 = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_LEFT)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_AUDIENCE)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        Action blueBackstageBotTeamPropLeftRoute4 = blah6.alignToLeftSideOfBackDropWithAprilTag();

        Action blueBackstageBotTeamPropLeftRoute5 = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_CENTER)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        blueBackstageBotTeamPropLeftRoute = new SequentialAction(
                blueBackstageBotTeamPropLeftRoute1,
                blueBackstageBotTeamPropLeftRoute2,
                blueBackstageBotTeamPropLeftRoute2a,
                blueBackstageBotTeamPropLeftRoute3,
                blueBackstageBotTeamPropLeftRoute2,
                blueBackstageBotTeamPropLeftRoute5
        );

        Action redBackstageBotTeamPropRightRoute1 = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_115_DEGREES)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .setReversed(true)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, FACE_TOWARD_BACKSTAGE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        CustomActions blah3 = new CustomActions();
        CustomActions blah4 = new CustomActions();
        Action redBackstageBotTeamPropRightRoute2 = blah3.alignToRightSideOfBackDropWithAprilTag();

        Action redBackstageBotTeamPropRightRoute2a = roadRunnerDrive.actionBuilder(RED_BACKDROP_CENTER)
                .build();

        Action redBackstageBotTeamPropRightRoute3 = roadRunnerDrive.actionBuilder(RED_BACKDROP_RIGHT)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_STAGING), TANGENT_TOWARD_AUDIENCE)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        Action redBackstageBotTeamPropRightRoute4 = blah4.alignToLeftSideOfBackDropWithAprilTag();

        Action redBackstageBotTeamPropRightRoute5 = roadRunnerDrive.actionBuilder(RED_BACKDROP_CENTER)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        redBackstageBotTeamPropRightRoute = new SequentialAction(
                redBackstageBotTeamPropRightRoute1,
                redBackstageBotTeamPropRightRoute2,
                redBackstageBotTeamPropRightRoute2a,
                redBackstageBotTeamPropRightRoute3,
                redBackstageBotTeamPropRightRoute2,
                redBackstageBotTeamPropRightRoute5
        );


        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_NEUTRAL_PIXEL_WING), FACE_TOWARD_BACKSTAGE), FACE_TOWARD_AUDIENCE)
                .afterDisp(3, CustomActions.pickupPixel())
                .lineToX(-40)
                .splineToLinearHeading(BLUE_AUDIENCE_ALIGNMENT,  0)

                .splineToConstantHeading(PoseToVector(BLUE_BACKSTAGE_ALIGNMENT), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .afterDisp(.5, CustomActions.dropPixelOnBackdrop())
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_BACKSTAGE_ALIGNMENT), TANGENT_TOWARD_AUDIENCE)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_ALIGNMENT), TANGENT_TOWARD_AUDIENCE)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_WING), TANGENT_TOWARD_AUDIENCE)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_ALIGNMENT), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKSTAGE_ALIGNMENT), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .afterDisp(.5, CustomActions.dropPixelOnBackdrop())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_45_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_WING), TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .lineToX(-40)
                .build();




        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_AUDIENCE_SPIKE_C), FACE_TOWARD_AUDIENCE), TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        Action redAudienceBotTeamPropLeftRoute1 = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_115_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        CustomActions blah = new CustomActions();
        CustomActions blah2 = new CustomActions();
        Action redAudienceBotTeamPropLeftRoute2 = blah.alignToLeftSideOfBackDropWithAprilTag();

        Action redAudienceBotTeamPropLeftRoute2a = roadRunnerDrive.actionBuilder(RED_BACKDROP_CENTER)
                .build();

        Action redAudienceBotTeamPropLeftRoute3 = roadRunnerDrive.actionBuilder(RED_BACKDROP_LEFT)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_STAGING), TANGENT_TOWARD_AUDIENCE)
                .splineToLinearHeading(RED_NEUTRAL_PIXEL_WING, TANGENT_TOWARD_AUDIENCE)
                .lineToX(RED_BACKDROP_STAGING.position.x)
                .build();

        Action redAudienceBotTeamPropLeftRoute4 = blah2.alignToLeftSideOfBackDropWithAprilTag();

        Action redAudienceBotTeamPropLeftRoute5 = roadRunnerDrive.actionBuilder(RED_BACKDROP_CENTER)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        redAudienceBotTeamPropLeftRoute = new SequentialAction(
                redAudienceBotTeamPropLeftRoute1,
                redAudienceBotTeamPropLeftRoute2,
                redAudienceBotTeamPropLeftRoute2a,
                redAudienceBotTeamPropLeftRoute3,
                redAudienceBotTeamPropLeftRoute2,
                redAudienceBotTeamPropLeftRoute5
        );


        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_225_DEGREES)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
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
        blueBackstageBot.runAction(RoutesSpikeStraightUpTheMiddle.blueBackstageBotTeamPropLeftRoute);
        redBackstageBot.runAction(RoutesSpikeStraightUpTheMiddle.redBackstageBotTeamPropLeftRoute);
        redAudienceBot.runAction(RoutesSpikeStraightUpTheMiddle.redAudienceBotTeamPropLeftRoute);
        blueAudienceBot.runAction(RoutesSpikeStraightUpTheMiddle.blueAudienceBotTeamPropLeftRoute);
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
