package com.example.meepmeeptesting.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static com.example.meepmeeptesting.Constants.AUDIENCE_ROBOT_WAIT_TIME;
import static com.example.meepmeeptesting.Constants.BACKSTAGE_ROBOT_WAIT_TIME;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SCORE_DISTANCE;
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
import static com.example.meepmeeptesting.MeepMeepTesting.*;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.meepmeeptesting.MeepMeepTesting;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

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

    public static double SLOW_VELOCITY_OVERRIDE = 10;
    public static double SLOW_ACCELERATION_OVERRIDE = 15;
    public static double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(45);

    public static double FAST_VELOCITY_OVERRIDE = 40;
    public static double FAST_ACCELERATION_OVERRIDE = 40;
    public static double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static double SUPER_FAST_VELOCITY_OVERRIDE = 40;
    public static double SUPER_FAST_ACCELERATION_OVERRIDE = 40;
    public static double SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;
    public static VelConstraint superFastVelocity;
    public static AccelConstraint superFastAcceleration;

    public static void BuildRoutes() {
        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);
        slowVelocity = new MinVelConstraint(Arrays.asList(
                new MecanumKinematics(15).new WheelVelConstraint(SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)));


        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);
        fastVelocity = new MinVelConstraint(Arrays.asList(
                new MecanumKinematics(15).new WheelVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)));

        superFastAcceleration = new ProfileAccelConstraint(-SUPER_FAST_ACCELERATION_OVERRIDE, SUPER_FAST_ACCELERATION_OVERRIDE);
        superFastVelocity = new MinVelConstraint(Arrays.asList(
                new MecanumKinematics(15).new WheelVelConstraint(SUPER_FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE)));


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
                .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.yellowPixelScorePose, posesForRouteSuper.yellowPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStagingByWall(posesForRouteSuper, posesForRouteSuper.yellowPixelScorePose))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .waitSeconds(BACKSTAGE_ROBOT_WAIT_TIME)
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteSuper, posesForRouteSuper.additionalWhitePixelScorePose))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalWhitePixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
//                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.additionalWhitePixelScorePose, posesForRouteSuper.parkPose))
                .waitSeconds(1.1)
                .build();
        return superBackstageAuto;
    }

    public Action SuperAudience(PosesForRouteSuper posesForRouteSuper) {
        Action superAudienceAuto = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .waitSeconds(AUDIENCE_ROBOT_WAIT_TIME)
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper,posesForRouteSuper.additionalWhitePixelScorePose))
                .stopAndAdd(new RouteBuilder().ScoreOnePixelAction(posesForRouteSuper.additionalWhitePixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.yellowPixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.yellowPixelScorePose, posesForRouteSuper.parkPose))
                .waitSeconds(1.1)
                .build();
        return superAudienceAuto;
    }

    public static class RouteBuilder {
        Action AutoDriveToBackDrop(Pose2d scorePose) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .lineToX(scorePose.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveToBackdrop;
        }

          public Action BackdropStagingToNeutralStagingByWall(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstagePathPose), TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audiencePathPose), TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.neutralStagingPose), posesForRouteSuper.neutralApproachOrientation, superFastVelocity, superFastAcceleration)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralStagingPose)
                    .setReversed(false)
                    .setTangent(posesForRouteSuper.leaveNeutralTangent)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audiencePathPose), posesForRouteSuper.backdropApproachOrientation, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstagePathPose), TANGENT_TOWARD_BACKSTAGE, superFastVelocity, superFastAcceleration)
                    .afterTime(1.1, ExtendLift(LiftStates.AUTO_LOW))
                    .splineToConstantHeading(PoseToVector(scorePose), TANGENT_TOWARD_BACKSTAGE, superFastVelocity, superFastAcceleration)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action NeutralStagingToBackdropStagingThroughStageDoor(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralStagingPose)
                    .setReversed(false)
                    .setTangent(posesForRouteSuper.leaveNeutralTangent)
                    .afterTime(.5, new TurnIntakeReverse())
                    .afterTime(1.5, new TurnIntakeOff())
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStageDoorPose), posesForRouteSuper.backdropApproachOrientation, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStageDoorPose), TANGENT_TOWARD_BACKSTAGE, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(scorePose), TANGENT_TOWARD_BACKSTAGE, superFastVelocity, superFastAcceleration)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action BackdropStagingToNeutralStagingThroughStageDoor(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStageDoorPose), TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStageDoorPose), TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.neutralStagingPose), posesForRouteSuper.neutralApproachOrientation, superFastVelocity, superFastAcceleration)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action PickupPixels(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ActuateGripperAction(GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    new RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, posesForRouteSuper.neutralPickupPose),
                    new TurnIntakeOn(),
                    new SleepAction(.1),
                    new RouteBuilder().AutoDriveFromNeutralStack(posesForRouteSuper));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRouteSuper posesForRouteSuper) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralPickupPose)
                    .splineToLinearHeading(posesForRouteSuper.neutralStagingPose, TANGENT_TOWARD_BACKSTAGE, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(Pose2d startPose, Pose2d endPose) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(endPose, TANGENT_TOWARD_AUDIENCE, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scoreStaging, LiftStates scoreHeight) {
            Action scorePixel = roadRunnerDrive.actionBuilder(scoreStaging)
                    .lineToX(scoreStaging.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .waitSeconds(.2)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.OPEN))
                    .waitSeconds(.4)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH))
                    .setReversed(true)
                    .afterTime(.8, RetractLift())
                    .lineToX(scoreStaging.position.x, slowVelocity, slowAcceleration)
                    .build();
            return scorePixel;
        }

        public Action ScoreOnePixelAction(Pose2d scoreStaging, LiftStates scoreHeight) {
            Action scorePixel = roadRunnerDrive.actionBuilder(scoreStaging)
                    .lineToX(scoreStaging.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .waitSeconds(.2)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.ONE_PIXEL_RELEASE_POSITION))
                    .waitSeconds(.4)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH))
                    .setReversed(true)
                    .afterTime(.8, RetractLift())
                    .lineToX(scoreStaging.position.x, slowVelocity, slowAcceleration)
                    .build();
            return scorePixel;

        }

        private Action PushTeamPropAndBackdropStage(PosesForRouteSuper posesForRouteSuper) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePose, posesForRouteSuper.spikePose.heading.log(), superFastVelocity, superFastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .afterTime(1.6, ExtendLift(LiftStates.AUTO_LOW))
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.yellowPixelScorePose, posesForRouteSuper.yellowPixelScorePose.heading.log(), superFastVelocity, superFastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteSuper posesForRouteSuper) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePose, posesForRouteSuper.spikePose.heading.log(), superFastVelocity, superFastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .setTangent(posesForRouteSuper.leaveSpikeTangent)
                    .splineToLinearHeading(posesForRouteSuper.neutralStagingPose, posesForRouteSuper.neutralApproachOrientation, superFastVelocity, superFastAcceleration)
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

            Action strafe = roadRunnerDrive.actionBuilder(posesForRouteSuper.additionalWhitePixelScorePose)
                    .strafeTo(PoseToVector(posesForRouteSuper.yellowPixelScorePose))
                    .build();
            return strafe;
        }


        public Action ExtendLift(LiftStates scoreHeight) {
            Action extendLift = new SequentialAction(
                    new ParallelAction(
                            new ActuateGripperAction(GripperStates.CLOSED),
                            new RotateShoulderAction(ShoulderStates.BACKDROP)),
                    new MoveLiftSlideActionFinishImmediate(scoreHeight));
            return extendLift;
        }

        public Action RetractLift() {
            return new SequentialAction(
                    new ParallelAction(
                            new ActuateGripperAction(GripperStates.CLOSED),
                            new RotateShoulderAction(ShoulderStates.HALFWAY),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.SAFE)
                    ),
                    new SleepAction(.25),
                    new RotateShoulderAction(ShoulderStates.INTAKE_VALUE_STAGING),
                    new MoveLiftSlideActionFinishImmediate(LiftStates.HOME),
                    new SleepAction(.25),
                    new RotateShoulderAction(ShoulderStates.INTAKE)
            );
        }

        private class ActuateGripperAction implements Action {
            public ActuateGripperAction(GripperStates open) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class ActuatePixelPusherAction implements Action {
            public ActuatePixelPusherAction(PixelPusherStates notPushing) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        private class MoveLiftSlideActionFinishImmediate implements Action {
            public MoveLiftSlideActionFinishImmediate(LiftStates scoreHeight) {
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

        private class RotateShoulderAction implements Action {
            public RotateShoulderAction(ShoulderStates state) {
            }

            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

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

