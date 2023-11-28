package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SCORE_DISTANCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
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
import static com.example.meepmeeptesting.MeepMeepTesting.AllianceColor.BLUE;
import static com.example.meepmeeptesting.MeepMeepTesting.AllianceColor.RED;
import static com.example.meepmeeptesting.MeepMeepTesting.SideOfField.AUDIENCE;
import static com.example.meepmeeptesting.MeepMeepTesting.SideOfField.BACKSTAGE;
import static com.example.meepmeeptesting.MeepMeepTesting.TeamPropLocation.CENTER;
import static com.example.meepmeeptesting.MeepMeepTesting.TeamPropLocation.LEFT;
import static com.example.meepmeeptesting.MeepMeepTesting.TeamPropLocation.RIGHT;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class RoutesSuper {
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
        PosesForRouteSuper blueBackstageLeftPoses = new PosesForRouteSuper(BLUE, BACKSTAGE, LEFT);
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(blueBackstageLeftPoses))
                .build();

        PosesForRouteSuper blueAudienceLeftPoses = new PosesForRouteSuper(BLUE, AUDIENCE, LEFT);
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(blueAudienceLeftPoses))
                .build();

        PosesForRouteSuper redBackstageLeftPoses = new PosesForRouteSuper(RED, BACKSTAGE, LEFT);
        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(redBackstageLeftPoses))
                .build();

        PosesForRouteSuper redAudienceLeftPoses = new PosesForRouteSuper(RED, AUDIENCE, LEFT);
        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(redAudienceLeftPoses))
                .build();

        ///////////
        // RIGHT //
        ///////////

        PosesForRouteSuper redBackstageRightPoses = new PosesForRouteSuper(RED, BACKSTAGE, RIGHT);
        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(redBackstageRightPoses))
                .build();

        PosesForRouteSuper redAudienceRightPoses = new PosesForRouteSuper(RED, AUDIENCE, RIGHT);
        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(redAudienceRightPoses))
                .build();

        PosesForRouteSuper blueBackstageRightPoses = new PosesForRouteSuper(BLUE, BACKSTAGE, RIGHT);
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(blueBackstageRightPoses))
                .build();

        PosesForRouteSuper blueAudienceRightPoses = new PosesForRouteSuper(BLUE, AUDIENCE, RIGHT);
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(blueAudienceRightPoses))
                .build();

        ////////////
        // CENTER //
        ////////////

        PosesForRouteSuper blueBackstageCenterPoses = new PosesForRouteSuper(BLUE, BACKSTAGE, CENTER);
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(blueBackstageCenterPoses))
                .build();

        PosesForRouteSuper blueAudienceCenterPoses = new PosesForRouteSuper(BLUE, AUDIENCE, CENTER);
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(blueAudienceCenterPoses))
                .build();

        PosesForRouteSuper redBackstageCenterPoses = new PosesForRouteSuper(RED, BACKSTAGE, CENTER);
        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperBackstage(redBackstageCenterPoses))
                .build();

        PosesForRouteSuper redAudienceCenterPoses = new PosesForRouteSuper(RED, AUDIENCE, CENTER);
        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .stopAndAdd(new RoutesSuper().SuperAudience(redAudienceCenterPoses))
                .build();
    }

    public Action SuperBackstage(PosesForRouteSuper posesForRouteSuper) {
        Action superBackstageAuto = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                .stopAndAdd(new RoutesSuper.RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().BackdropStagingToNeutralStagingByWall(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
                .stopAndAdd(new RoutesSuper.RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RoutesSuper.RouteBuilder().NeutralStagingToBackdropStagingByWall(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().Park(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper.parkPose))
                .build();
        return superBackstageAuto;
    }

    public Action SuperAudience(PosesForRouteSuper posesForRouteSuper) {
        Action superAudienceAuto = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                .stopAndAdd(new RoutesSuper.RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RoutesSuper.RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper,posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RoutesSuper.RouteBuilder().ScoreOnePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().BackdropStagingToNeutralStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.secondNeutralStagingPose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RoutesSuper.RouteBuilder().Park(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.parkPose))
                .build();
        return superAudienceAuto;
    }

    public static class RouteBuilder {
        Action AutoDriveToBackDrop(Pose2d scorePose, PosesForRouteSuper posesForRouteSuper) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .lineToX(scorePose.position.x+SCORE_DISTANCE)
                    .build();
            return autoDriveToBackdrop;
        }

        Action AutoDriveFromBackDrop(Pose2d scorePose, PosesForRouteSuper posesForRouteSuper) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .lineToX(scorePose.position.x)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStagingByWall(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStartPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStartPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.neutralStagingPose), posesForRouteSuper.neutralApproachOrientation)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStagingByWall(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralStagingPose)
                    .setReversed(false)
                    .setTangent(posesForRouteSuper.leaveNeutralTangent)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStartPose), posesForRouteSuper.backdropApproachOrientation)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStartPose), TANGENT_TOWARD_BACKSTAGE)
                    .splineToConstantHeading(PoseToVector(scorePose), TANGENT_TOWARD_BACKSTAGE)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action NeutralStagingToBackdropStagingThroughStageDoor(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralStagingPose)
                    .setReversed(false)
                    .setTangent(posesForRouteSuper.leaveNeutralTangent)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStageDoorPose), posesForRouteSuper.backdropApproachOrientation)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStageDoorPose), TANGENT_TOWARD_BACKSTAGE)
                    .splineToConstantHeading(PoseToVector(scorePose), TANGENT_TOWARD_BACKSTAGE)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action BackdropStagingToNeutralStagingThroughStageDoor(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStageDoorPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStageDoorPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.neutralStagingPose), posesForRouteSuper.neutralApproachOrientation)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action PickupPixels(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ParallelAction(
                            new RobotCommands().TurnIntakeOn(),
                            new RoutesSuper.RouteBuilder().AutoDriveToNeutralStack(posesForRouteSuper, neutralPixelStagingPose)),
                    new SleepAction(.1),
                    new ParallelAction(
                            new RobotCommands().TurnIntakeOff(),
                            new RoutesSuper.RouteBuilder().AutoDriveFromNeutralStack(posesForRouteSuper)));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRouteSuper posesForRouteSuper) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralPickupPose)
                    .lineToX(posesForRouteSuper.neutralStagingPose.position.x)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(neutralPixelStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRouteSuper.neutralPickupPose.position.x)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scorePose, PosesForRouteSuper posesForRouteSuper) {
            SequentialAction scorePixel = new SequentialAction(
                    new ParallelAction(
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRouteSuper),
                            new RobotCommands().LiftLow(),
                            new RobotCommands().RotateShoulderToBackdrop()),
                    new SleepAction(.2),
                    new RobotCommands().OpenClaw(),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRouteSuper),
                            new RobotCommands().CloseClaw(),
                            new RobotCommands().RotateShoulderToIntake()),
                    new RobotCommands().LiftHome()
            );
            return scorePixel;
        }

        public Action ScoreOnePixelAction(Pose2d scorePose, PosesForRouteSuper posesForRouteSuper) {
            SequentialAction scorePixel = new SequentialAction(
                    new ParallelAction(
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRouteSuper),
                            new RobotCommands().LiftLow(),
                            new RobotCommands().RotateShoulderToBackdrop()),
                    new SleepAction(.2),
                    new RobotCommands().OpenClawHalfway(),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRouteSuper),
                            new RobotCommands().CloseClaw(),
                            new RobotCommands().RotateShoulderToIntake()),
                    new RobotCommands().LiftHome()
            );
            return scorePixel;
        }

        private Action PushTeamPropAndBackdropStage(PosesForRouteSuper posesForRouteSuper) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePosePast, posesForRouteSuper.spikePosePast.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.spikePoseDrop, -posesForRouteSuper.spikePoseDrop.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.firstPixelScorePose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteSuper posesForRouteSuper) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .setTangent(posesForRouteSuper.startingTangent)
                    .splineToLinearHeading(posesForRouteSuper.spikePosePast, posesForRouteSuper.spikePosePast.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.spikePoseDrop, -posesForRouteSuper.spikePoseDrop.heading.log())
                    .setReversed(true)
                    .setTangent(posesForRouteSuper.leaveSpikeTangent)
                    .splineToLinearHeading(posesForRouteSuper.neutralStagingPose, posesForRouteSuper.neutralApproachOrientation)
                    .build();
            return pushTeamPropAndStage;
        }

        private Action Park(Pose2d startPose, Pose2d parkPose) {
            Action park = roadRunnerDrive.actionBuilder(startPose)
                    .strafeTo(PoseToVector(parkPose))
                    .build();
            return park;
        }

        public Action StrafeToPlaceFirstPixel(PosesForRouteSuper posesForRouteSuper) {

            Action strafe = roadRunnerDrive.actionBuilder(posesForRouteSuper.additionalPixelScorePose)
                    .strafeTo(PoseToVector(posesForRouteSuper.firstPixelScorePose))
                    .build();
            return strafe;
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

