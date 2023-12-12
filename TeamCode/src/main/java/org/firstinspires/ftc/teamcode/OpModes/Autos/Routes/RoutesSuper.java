package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem.*;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.SideOfField.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.TeamPropLocation.*;

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

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlow;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlowReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.ActuatePixelPusherAction;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Poses.PosesForRouteSuper;

import java.util.Arrays;


public class RoutesSuper {
    static MecanumDriveMona roadRunnerDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

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

    public static Action blueTestRoute;

    public static void BuildRoutes() {

        MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        slowVelocity = new MinVelConstraint(Arrays.asList(
                        mecanumDrive.kinematics.new WheelVelConstraint(SLOW_VELOCITY_OVERRIDE),
                        new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)));
        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                mecanumDrive.kinematics.new WheelVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)));
        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);

        superFastVelocity = new MinVelConstraint(Arrays.asList(
                mecanumDrive.kinematics.new WheelVelConstraint(SUPER_FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE)));
        superFastAcceleration = new ProfileAccelConstraint(-SUPER_FAST_ACCELERATION_OVERRIDE, SUPER_FAST_ACCELERATION_OVERRIDE);


        PosesForRouteSuper blueTestRoutePoses = new PosesForRouteSuper(BLUE, AUDIENCE, CENTER);
        blueTestRoute = roadRunnerDrive.actionBuilder(BLUE_NEUTRAL_STAGING)
                .stopAndAdd(new RouteBuilder().PickupPixels(blueTestRoutePoses, blueTestRoutePoses.neutralStagingPose))
                .build();

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
                .waitSeconds(1)
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
                .waitSeconds(1)
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

        Action AutoDriveFromBackDrop(Pose2d scorePose) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .lineToX(scorePose.position.x, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveFromBackdrop;
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
                    .splineToConstantHeading(PoseToVector(scorePose), TANGENT_TOWARD_BACKSTAGE, superFastVelocity, superFastAcceleration)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action NeutralStagingToBackdropStagingThroughStageDoor(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralStagingPose)
                    .setReversed(false)
                    .setTangent(posesForRouteSuper.leaveNeutralTangent)
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


        public Action PickupPixelsTest2(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, posesForRouteSuper.neutralPickupPose),
                    new SleepAction(1),
                    new TurnIntakeSlow(),
                    new SleepAction(1),
                    new TurnIntakeReverse(),
                    new SleepAction(1),
                    new TurnIntakeOn(),
                    new RouteBuilder().AutoDriveFromNeutralStack(posesForRouteSuper));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRouteSuper posesForRouteSuper) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(posesForRouteSuper.neutralPickupPose)
                    .lineToX(posesForRouteSuper.neutralStagingPose.position.x, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(Pose2d startPose, Pose2d endPose) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(true)
                    .lineToX(endPose.position.x, slowVelocity, slowAcceleration)
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
                                            new SleepAction(.2),
                                            new MoveLiftSlideActionFinishImmediate(scoreHeight)
                                    ),
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose),
                            new SleepAction(.45),
                            new ActuateGripperAction(GripperStates.OPEN),
                            new SleepAction(.55),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.3),
                            new ParallelAction(
                                    new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.6),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperStates.CLOSED),
                                                    new MoveLiftSlideActionFinishImmediate(LiftStates.SAFE)
                                            ),
                                            new SleepAction(.4),
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
                                            new SleepAction(.4),
                                            new MoveLiftSlideActionFinishImmediate(LiftStates.HOME),
                                            new SleepAction(.25),
                                            new RotateShoulderAction(ShoulderStates.INTAKE)
                                    )
                            )
                    );
            return scorePixel;
        }

        private Action PushTeamPropAndBackdropStage(PosesForRouteSuper posesForRouteSuper) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePose, posesForRouteSuper.spikePose.heading.log(), superFastVelocity, superFastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
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

    }
}

