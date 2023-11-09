package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
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
import static com.example.meepmeeptesting.MeepMeepTesting.AllianceColor.*;
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
    private static Action redAudienceBotTeamPropLeftRoute;
    private static Action blueBackstageBotTeamPropLeftRoute;

    //Variables to store routes for team prop right for all four start locations
    private static Action redBackstageBotTeamPropRightRoute;
    private static Action redAudienceBotTeamPropRightRoute;
    private static Action blueBackstageBotTeamPropRightRoute;
    private static Action blueAudienceBotTeamPropRightRoute;


    public static void BuildRoutes() {

        /** BLUE BACKSTAGE RIGHT **/
        PosesForRoute blueBackstageRightPoses = new PosesForRoute(BLUE, BACKSTAGE, RIGHT);
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageRightPoses))
                .build();

        /** RED BACKSTAGE LEFT **/
        PosesForRoute redBackstageRightPoses = new PosesForRoute(RED, BACKSTAGE, RIGHT);
        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageRightPoses))
                .build();

        /** BLUE AUDIENCE RIGHT **/
        PosesForRoute blueAudienceRightPoses = new PosesForRoute(BLUE, AUDIENCE, RIGHT);
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceRightPoses))
                .build();

        /** RED AUDIENCE RIGHT **/
        PosesForRoute redAudienceRightPoses = new PosesForRoute(RED, AUDIENCE, RIGHT);
        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceRightPoses))
                .build();

        /** BLUE BACKSTAGE CENTER **/
        PosesForRoute blueBackstageCenterPoses = new PosesForRoute(BLUE, BACKSTAGE, CENTER);
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageCenterPoses))
                .build();

        /** RED BACKSTAGE CENTER **/
        PosesForRoute redBackstageCenterPoses = new PosesForRoute(RED, BACKSTAGE, CENTER);
        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageCenterPoses))
                .build();

        /** BLUE AUDIENCE CENTER **/
        PosesForRoute blueAudienceCenterPoses = new PosesForRoute(BLUE, AUDIENCE, CENTER);
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceCenterPoses))
                .build();

        /** RED AUDIENCE CENTER **/
        PosesForRoute redAudienceCenterPoses = new PosesForRoute(RED, AUDIENCE, CENTER);
        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceCenterPoses))
                .build();

        /** BLUE BACKSTAGE LEFT **/
        PosesForRoute blueBackstageLeftPoses = new PosesForRoute(BLUE, BACKSTAGE, LEFT);
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageLeftPoses))
                .build();

        /** RED BACKSTAGE LEFT **/
        PosesForRoute redBackstageLeftPoses = new PosesForRoute(RED, BACKSTAGE, LEFT);
        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageLeftPoses))
                .build();


        /** BLUE AUDIENCE LEFT **/
        PosesForRoute blueAudienceLeftPoses = new PosesForRoute(BLUE, AUDIENCE, LEFT);
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceLeftPoses))
                .build();

        /** RED AUDIENCE LEFT **/
        PosesForRoute redAudienceLeftPoses = new PosesForRoute(RED, AUDIENCE, LEFT);
        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceLeftPoses))
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

    public static class RouteBuilder {
        Action AutoDriveToBackDrop(Pose2d scorePose, PosesForRoute posesForRoute) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .splineToLinearHeading(scorePose, TANGENT_TOWARD_BACKSTAGE)
                    .build();
            return autoDriveToBackdrop;
        }

        Action AutoDriveFromBackDrop(Pose2d scorePose, PosesForRoute posesForRoute) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRoute.backdropStagingPose), TANGENT_TOWARD_AUDIENCE)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(PosesForRoute posesForRoute) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRoute.neutralStagingPose.position.x)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(PosesForRoute posesForRoute) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRoute.neutralStagingPose)
                    .lineToX(posesForRoute.backdropStagingPose.position.x)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action PickupPixels(PosesForRoute posesForRoute) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ParallelAction(
                            new CustomActions().TurnIntakeOn(),
                            new RouteBuilder().AutoDriveToNeutralStack(posesForRoute)),
                    new SleepAction(.1),
                    new ParallelAction(
                            new CustomActions().TurnIntakeOff(),
                            new RouteBuilder().AutoDriveFromNeutralStack(posesForRoute)));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRoute posesForRoute) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(posesForRoute.neutralPickupPose)
                    .lineToX(posesForRoute.neutralStagingPose.position.x)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(PosesForRoute posesForRoute) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(posesForRoute.neutralStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRoute.neutralPickupPose.position.x)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scorePose, PosesForRoute posesForRoute) {
            SequentialAction scorePixel = new SequentialAction(
                    new ParallelAction(
                            new RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRoute),
                            new CustomActions().LiftLow(),
                            new CustomActions().RotateShoulderToBackdrop()),
                    new SleepAction(.2),
                    new CustomActions().CloseClaw(),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRoute),
                            new CustomActions().CloseClaw(),
                            new CustomActions().RotateShoulderToIntake()),
                    new CustomActions().LiftHome()
            );
            return scorePixel;
        }



        private Action PushTeamPropAndBackdropStage(PosesForRoute posesForRoute) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.spikePose, posesForRoute.spikePose.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRoute.backdropStagingPose, posesForRoute.backdropStagingPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRoute posesForRoute) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.spikePose, posesForRoute.spikePose.heading.log())
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRoute.neutralStagingPose), posesForRoute.neutralPickupPose.heading.log())
                    .turnTo(posesForRoute.neutralPickupPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action Park(PosesForRoute posesForRoute) {
            Action park = roadRunnerDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .strafeTo(PoseToVector(posesForRoute.parkPose))
                    .turnTo(posesForRoute.parkOrientation)
                    .build();
            return park;
        }

        public Action PushPropScoreFive(PosesForRoute posesForRoute) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.firstPixelScorePose, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRoute posesForRoute) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
            return pushPropScoreFive;
        }


    }
}

