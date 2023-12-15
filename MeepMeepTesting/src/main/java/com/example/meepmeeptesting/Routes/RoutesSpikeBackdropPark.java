package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;
import static com.example.meepmeeptesting.Constants.*;
import static com.example.meepmeeptesting.MeepMeepTesting.*;
import static com.example.meepmeeptesting.MeepMeepTesting.LiftStates.*;
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
import static com.example.meepmeeptesting.MeepMeepTesting.LiftStates.AUTO_LOW;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import com.acmerobotics.roadrunner.Action;

public class RoutesSpikeBackdropPark {
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

    public static void BuildRoutes() {

        Action dropPurple = new SleepAction(.1);

        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(11)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_RIGHT,AUTO_HIGH, AUTO_MID))
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE) //WAS Redaudiencespike Right and tangent toward blue
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(14)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();
    }

    public static class ActionsForSpikeBackdrop {
        public Action ScoreAndBackup(Pose2d start_pose, LiftStates liftHeight) {
            return roadRunnerDrive.actionBuilder(start_pose)
//                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(liftHeight))
                    .waitSeconds(.5)
                    .lineToX(start_pose.position.x + 5.5)
                    .waitSeconds(.9)
//                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
//                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.AUTO_MID))
                    .waitSeconds(.5)
                    .lineToX(start_pose.position.x)
//                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }

        public Action ScoreWithTwoHeightsAndBackup(Pose2d start_pose, LiftStates firstHeight, LiftStates secondHeight) {
            return roadRunnerDrive.actionBuilder(start_pose)
//                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(firstHeight))
                    .waitSeconds(.2)
                    .lineToX(start_pose.position.x + 5.5)
                    .waitSeconds(.2)
//                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(secondHeight))
                    .waitSeconds(.2)
//                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
//                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(firstHeight))
                    .waitSeconds(.3)
                    .lineToX(start_pose.position.x)
//                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
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

