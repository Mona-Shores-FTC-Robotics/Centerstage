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

public class RoutesSpikeBackdropParkAutoBuild {
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
        PosesForRouteStraight blueBackstageRightPoses = new PosesForRouteStraight(BLUE, BACKSTAGE, RIGHT);
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageRightPoses))
                .build();

        /** RED BACKSTAGE LEFT **/
        PosesForRouteStraight redBackstageRightPoses = new PosesForRouteStraight(RED, BACKSTAGE, RIGHT);
        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageRightPoses))
                .build();

        /** BLUE AUDIENCE RIGHT **/
        PosesForRouteStraight blueAudienceRightPoses = new PosesForRouteStraight(BLUE, AUDIENCE, RIGHT);
        blueAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(blueAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceRightPoses))
                .build();

        /** RED AUDIENCE RIGHT **/
        PosesForRouteStraight redAudienceRightPoses = new PosesForRouteStraight(RED, AUDIENCE, RIGHT);
        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(redAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceRightPoses))
                .build();

        /** BLUE BACKSTAGE CENTER **/
        PosesForRouteStraight blueBackstageCenterPoses = new PosesForRouteStraight(BLUE, BACKSTAGE, CENTER);
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageCenterPoses))
                .build();

        /** RED BACKSTAGE CENTER **/
        PosesForRouteStraight redBackstageCenterPoses = new PosesForRouteStraight(RED, BACKSTAGE, CENTER);
        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageCenterPoses))
                .build();

        /** BLUE AUDIENCE CENTER **/
        PosesForRouteStraight blueAudienceCenterPoses = new PosesForRouteStraight(BLUE, AUDIENCE, CENTER);
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(blueAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceCenterPoses))
                .build();

        /** RED AUDIENCE CENTER **/
        PosesForRouteStraight redAudienceCenterPoses = new PosesForRouteStraight(RED, AUDIENCE, CENTER);
        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(redAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceCenterPoses))
                .build();

        /** BLUE BACKSTAGE LEFT **/
        PosesForRouteStraight blueBackstageLeftPoses = new PosesForRouteStraight(BLUE, BACKSTAGE, LEFT);
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageLeftPoses))
                .build();

        /** RED BACKSTAGE LEFT **/
        PosesForRouteStraight redBackstageLeftPoses = new PosesForRouteStraight(RED, BACKSTAGE, LEFT);
        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageLeftPoses))
                .build();


        /** BLUE AUDIENCE LEFT **/
        PosesForRouteStraight blueAudienceLeftPoses = new PosesForRouteStraight(BLUE, AUDIENCE, LEFT);
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(blueAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceLeftPoses))
                .build();

        /** RED AUDIENCE LEFT **/
        PosesForRouteStraight redAudienceLeftPoses = new PosesForRouteStraight(RED, AUDIENCE, LEFT);
        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(redAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceLeftPoses))
                .build();
    }

    public static class RouteBuilder {
        Action AutoDriveToBackDrop(Pose2d scorePose, PosesForRouteStraight posesForRouteStraight) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(posesForRouteStraight.backdropStagingPose)
                    .splineToLinearHeading(scorePose, TANGENT_TOWARD_BACKSTAGE)
                    .build();
            return autoDriveToBackdrop;
        }

        Action AutoDriveFromBackDrop(Pose2d scorePose, PosesForRouteStraight posesForRouteStraight) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteStraight.backdropStagingPose), TANGENT_TOWARD_AUDIENCE)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(PosesForRouteStraight posesForRouteStraight) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(posesForRouteStraight.backdropStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRouteStraight.neutralStagingPose.position.x)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(PosesForRouteStraight posesForRouteStraight) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteStraight.neutralStagingPose)
                    .lineToX(posesForRouteStraight.backdropStagingPose.position.x)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action PickupPixels(PosesForRouteStraight posesForRouteStraight) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ParallelAction(
                            new RobotCommands().TurnIntakeOn(),
                            new RouteBuilder().AutoDriveToNeutralStack(posesForRouteStraight)),
                    new SleepAction(.1),
                    new ParallelAction(
                            new RobotCommands().TurnIntakeOff(),
                            new RouteBuilder().AutoDriveFromNeutralStack(posesForRouteStraight)));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRouteStraight posesForRouteStraight) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(posesForRouteStraight.neutralPickupPose)
                    .lineToX(posesForRouteStraight.neutralStagingPose.position.x)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(PosesForRouteStraight posesForRouteStraight) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(posesForRouteStraight.neutralStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRouteStraight.neutralPickupPose.position.x)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scorePose, PosesForRouteStraight posesForRouteStraight) {
            SequentialAction scorePixel = new SequentialAction(
                    new ParallelAction(
                            new RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRouteStraight),
                            new RobotCommands().LiftLow(),
                            new RobotCommands().RotateShoulderToBackdrop()),
                    new SleepAction(.2),
                    new RobotCommands().OpenClaw(),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRouteStraight),
                            new RobotCommands().CloseClaw(),
                            new RobotCommands().RotateShoulderToIntake()),
                    new RobotCommands().LiftHome()
            );
            return scorePixel;
        }



        private Action PushTeamPropAndBackdropStage(PosesForRouteStraight posesForRouteStraight) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .splineToLinearHeading(posesForRouteStraight.spikePose, posesForRouteStraight.spikePose.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteStraight.backdropStagingPose, posesForRouteStraight.backdropStagingPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteStraight posesForRouteStraight) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .splineToLinearHeading(posesForRouteStraight.spikePose, posesForRouteStraight.spikePose.heading.log())
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteStraight.neutralStagingPose), posesForRouteStraight.neutralPickupPose.heading.log())
                    .turnTo(posesForRouteStraight.neutralPickupPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action Park(PosesForRouteStraight posesForRouteStraight) {
            Action park = roadRunnerDrive.actionBuilder(posesForRouteStraight.backdropStagingPose)
                    .strafeTo(PoseToVector(posesForRouteStraight.parkPose))
                    .turnTo(posesForRouteStraight.parkOrientation)
                    .build();
            return park;
        }

        public Action PushPropScoreFive(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.firstPixelScorePose, posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
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

