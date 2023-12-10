package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.SideOfField.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.TeamPropLocation.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlow;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlowReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.ActuatePixelPusherAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem;
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

    public static Action blueTestRoute;

    public enum LiftStates {
        AUTO_LOW, AUTO_MID, AUTO_HIGH, MAX, HIGH, MID, LOW, SAFE, HOME, ZERO, MANUAL;
    }

    public static VelConstraint overrideVelConstraint;
    public static AccelConstraint overrideAccelConstraint;
    public static TurnConstraints overrideTurnConstraint;

    public static VelConstraint overrideVelConstraint2;
    public static AccelConstraint overrideAccelConstraint2;

    public static double VELOCITY_OVERRIDE = 10;
    public static double ACCELERATION_OVERRIDE = 15;
    public static double TURN_OVERRIDE=30;

    public static void BuildRoutes() {

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(VELOCITY_OVERRIDE),
                        new AngularVelConstraint(VELOCITY_OVERRIDE)
                ));


        overrideAccelConstraint = new ProfileAccelConstraint(-ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

        overrideVelConstraint2 =
                new MinVelConstraint(Arrays.asList(
                        Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(8),
                        new AngularVelConstraint(8)
                ));

        overrideAccelConstraint2 = new ProfileAccelConstraint(-10, 10);



        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(30), -Math.toRadians(TURN_OVERRIDE), Math.toRadians(TURN_OVERRIDE));


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
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.firstPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStagingByWall(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper.parkPose))
                .waitSeconds(1)
                .build();
        return superBackstageAuto;
    }

    public Action SuperAudience(PosesForRouteSuper posesForRouteSuper) {
        Action superAudienceAuto = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper,posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RouteBuilder().ScoreOnePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
                .stopAndAdd(new RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.additionalPixelPixelScoreHeight))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().BackdropStagingToNeutralStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.secondNeutralStagingPose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.parkPose))
                .build();
        return superAudienceAuto;
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

        public Action BackdropStagingToNeutralStagingByWall(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.backstageStartPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.audienceStartPose), TANGENT_TOWARD_AUDIENCE)
                    .splineToConstantHeading(PoseToVector(posesForRouteSuper.neutralStagingPose), posesForRouteSuper.neutralApproachOrientation)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(PosesForRouteSuper posesForRouteSuper, Pose2d scorePose) {
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

        public Action PickupPixelsOLD(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new TurnIntakeOn(),
                    new RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, posesForRouteSuper.neutralPickupPose),
                    new SleepAction(.5),
                    new ParallelAction(
                            new TurnIntakeOff(),
                            new RouteBuilder().AutoDriveFromNeutralStack(posesForRouteSuper)));
            return pickupPixels;
        }

        public Action PickupPixels(PosesForRouteSuper posesForRouteSuper, Pose2d neutralPixelStagingPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    //we need to be closer to the blue wall by like 2-5 inches and maybe drive in just a tad more
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
                    .lineToX(posesForRouteSuper.neutralStagingPose.position.x, overrideVelConstraint, overrideAccelConstraint)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(Pose2d startPose, Pose2d endPose) {
            Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(true)
                    .lineToX(endPose.position.x, overrideVelConstraint, overrideAccelConstraint)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scorePose, LiftSlideSubsystem.LiftStates scoreHeight) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                            new SleepAction(.2),
                            new SequentialAction(
                                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP),
                                            new SleepAction(.35),
                                            new MoveLiftSlideActionFinishImmediate(scoreHeight)
                                    ),
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose),
                            new SleepAction(.4),
                            new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN),
                            new SleepAction(.4),
                            new MoveLiftSlideActionFinishImmediate(AUTO_HIGH),
                            new SleepAction(.8),
                            new ParallelAction(
                                    new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.9),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                                                    new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE)
                                            ),
                                            new SleepAction(.8),
                                            new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.HOME),
                                            new SleepAction(.250),
                                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE)
                                    )
                            )
                    );
            return scorePixel;
        }

        public Action ScoreOnePixelAction(Pose2d scorePose, LiftSlideSubsystem.LiftStates scoreHeight) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                            new SleepAction(.2),
                            new SequentialAction(
                                    new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP),
                                    new SleepAction(.35),
                                    new MoveLiftSlideActionFinishImmediate(scoreHeight)
                            ),
                            new RoutesSuper.RouteBuilder().AutoDriveToBackDrop(scorePose),
                            new SleepAction(.4),
                            new ActuateGripperAction(GripperSubsystem.GripperStates.ONE_PIXEL_RELEASE_POSITION),
                            new SleepAction(.4),
                            new MoveLiftSlideActionFinishImmediate(AUTO_HIGH),
                            new SleepAction(.8),
                            new ParallelAction(
                                    new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.9),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                                                    new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.SAFE)
                                            ),
                                            new SleepAction(.8),
                                            new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.HOME),
                                            new SleepAction(.25),
                                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE)
                                    )
                            )
                    );
            return scorePixel;
        }

        private Action PushTeamPropAndBackdropStage(PosesForRouteSuper posesForRouteSuper) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePose, posesForRouteSuper.spikePose.heading.log())
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.firstPixelScorePose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteSuper posesForRouteSuper) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);

            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePose, posesForRouteSuper.spikePose.heading.log())
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
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
}

