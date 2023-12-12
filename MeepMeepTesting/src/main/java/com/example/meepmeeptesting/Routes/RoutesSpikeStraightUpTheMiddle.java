package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.MeepMeepTesting.*;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.SCORE_DISTANCE;
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

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import org.jetbrains.annotations.NotNull;

public class RoutesSpikeStraightUpTheMiddle {
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

    public static VelConstraint overrideVelConstraint;
    public static AccelConstraint overrideAccelConstraint;

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
        Action AutoDriveToBackDrop(Pose2d scorePose) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .lineToX(scorePose.position.x+SCORE_DISTANCE)
                    .build();
            return autoDriveToBackdrop;
        }

        Action AutoDriveFromBackDrop(Pose2d scorePose) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .lineToX(scorePose.position.x)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(Pose2d scorePose, Pose2d neutralStagingPose, double approachTangent) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .setTangent(approachTangent)
                    .splineToLinearHeading(neutralStagingPose, TANGENT_TOWARD_AUDIENCE)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(Pose2d startPose, Pose2d endPose, double approachTangent) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(startPose)
                    .setTangent(TANGENT_TOWARD_BACKSTAGE)
                    .setReversed(false)
                    .splineToConstantHeading(PoseToVector(endPose), approachTangent)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.CLOSED))
                    .stopAndAdd(new TurnIntakeOff())
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action PickupPixels(Pose2d neutralPixelStagingPose, Pose2d neutralPickupPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ActuateGripperAction(GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, neutralPickupPose),
                    new TurnIntakeOn(),
                    new SleepAction(.1),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveFromNeutralStack(neutralPickupPose, neutralPixelStagingPose));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(Pose2d startPose, Pose2d endPose) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(false)
                    .lineToX(endPose.position.x)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(Pose2d startPose, Pose2d endPose) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(endPose), TANGENT_TOWARD_AUDIENCE)
//                    .lineToX(endPose.position.x, overrideVelConstraint, overrideAccelConstraint)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scorePose, LiftStates scoreHeight) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ActuateGripperAction(GripperStates.CLOSED),
                            new SleepAction(.2),
                            new SequentialAction(
                                    new RotateShoulderAction(ShoulderStates.BACKDROP),
                                    new SleepAction(.35),
                                    new MoveLiftSlideActionFinishImmediate(scoreHeight)
                            ),
                            new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToBackDrop(scorePose),
                            new SleepAction(.6),
                            new ActuateGripperAction(GripperStates.OPEN),
                            new SleepAction(.7),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.7),
                            new ParallelAction(
                                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.4),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperStates.CLOSED),
                                                    new MoveLiftSlideActionFinishImmediate(LiftStates.SAFE)
                                            ),
                                            new SleepAction(.5),
                                            new MoveLiftSlideActionFinishImmediate(LiftStates.HOME),
                                            new SleepAction(.250),
                                            new RotateShoulderAction(ShoulderStates.INTAKE)
                                    )
                            )
                    );
            return scorePixel;
        }


        public Action ScoreOnePixelAction(Pose2d scorePose, LiftStates scoreHeight) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ActuateGripperAction(GripperStates.CLOSED),
                            new SleepAction(.2),
                            new SequentialAction(
                                    new RotateShoulderAction(ShoulderStates.BACKDROP),
                                    new SleepAction(.35),
                                    new MoveLiftSlideActionFinishImmediate(scoreHeight)
                            ),
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose),
                            new SleepAction(.4),
                            new ActuateGripperAction(GripperStates.ONE_PIXEL_RELEASE_POSITION),
                            new SleepAction(.4),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.8),
                            new ParallelAction(
                                    new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.9),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperStates.CLOSED),
                                                    new MoveLiftSlideActionFinishImmediate(LiftStates.SAFE)
                                            ),
                                            new SleepAction(.8),
                                            new MoveLiftSlideActionFinishImmediate(LiftStates.HOME),
                                            new SleepAction(.25),
                                            new RotateShoulderAction(ShoulderStates.INTAKE)
                                    )
                            )
                    );
            return scorePixel;
        }

        public Action StrafeToPlaceFirstPixel(Pose2d startPose, Pose2d endPose) {
            Action strafe = roadRunnerDrive.actionBuilder(startPose)
                    .strafeTo(PoseToVector(endPose))
                    .build();
            return strafe;
        }

        private Action PushTeamPropAndBackdropStage(Pose2d startPose, Pose2d spikePose, Pose2d scorePose) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(scorePose, scorePose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteStraight posesForRouteStraight) {
            Action dropPurple = new ActuateGripperAction(GripperStates.CLOSED);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .splineToLinearHeading(posesForRouteStraight.spikePose, posesForRouteStraight.spikePose.heading.log())
                    .stopAndAdd(dropPurple)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteStraight.neutralStagingPose), posesForRouteStraight.neutralPickupPose.heading.log())
                    .stopAndAdd(new RotateShoulderAction(ShoulderStates.INTAKE))
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
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteStraight.startingPose, posesForRouteStraight.spikePose, posesForRouteStraight.yellowPixelScorePose))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.yellowPixelScoreHeight))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.neutralStagingPose, posesForRouteStraight.yellowPixelLeaveTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralStagingPose, posesForRouteStraight.neutralPickupPose))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight))
//                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.neutralStagingPose, posesForRouteStraight.additionalPixelScorePoseLeaveTangent))
//                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralStagingPose, posesForRouteStraight.neutralPickupPose))
//                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
//                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight))
//                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralStagingPose, posesForRouteStraight.neutralPickupPose))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScoreOnePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight))
                    .stopAndAdd(new RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.yellowPixelScorePose))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.neutralStagingPose, posesForRouteStraight.yellowPixelLeaveTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralStagingPose, posesForRouteStraight.neutralPickupPose))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight))
                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
        }
        ////////////////////////////////
        // PLACEHOLDER ACTIONS        //
        ///////////////////////////////

        private class TurnIntakeOn implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class TurnIntakeOff implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class RotateShoulderAction implements Action {
            public RotateShoulderAction(MeepMeepTesting.ShoulderStates state) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class ActuateGripperAction implements Action {
            public ActuateGripperAction(MeepMeepTesting.GripperStates open) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class ActuatePixelPusherAction implements Action {
            public ActuatePixelPusherAction(MeepMeepTesting.PixelPusherStates notPushing) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class MoveLiftSlideActionFinishImmediate implements Action {
            public MoveLiftSlideActionFinishImmediate(MeepMeepTesting.LiftStates scoreHeight) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class TurnIntakeSlowReverse implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class TurnIntakeSlow implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class TurnIntakeReverse implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
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

