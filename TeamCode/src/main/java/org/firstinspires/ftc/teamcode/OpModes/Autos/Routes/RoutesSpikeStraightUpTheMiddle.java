package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.*;
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
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlowReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.ActuatePixelPusherAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Poses.PosesForRouteStraight;

import java.util.Arrays;


public class RoutesSpikeStraightUpTheMiddle {
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

    public static double SUPER_FAST_VELOCITY_OVERRIDE = 55;
    public static double SUPER_FAST_ACCELERATION_OVERRIDE = 55;
    public static double SUPER_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;
    public static VelConstraint superFastVelocity;
    public static AccelConstraint superFastAcceleration;
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
        public Action PushPropScoreFive(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteStraight.startingPose, posesForRouteStraight.spikePose, posesForRouteStraight.yellowPixelScorePose))
                    .stopAndAdd(new RouteBuilder().ScorePixelAndNeutralPixelStage(
                            posesForRouteStraight.yellowPixelScorePose,
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.approachTrussStagingFromIntermediateTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixelsConstantHeading(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.neutralTrussPickupPose,
                            posesForRouteStraight.approachTrussPickupFromStagingTangent,
                            posesForRouteStraight.intermediatePose,
                            posesForRouteStraight.approachIntermediateStagingFromPickupTangent,
                            posesForRouteStraight.backdropStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAndNeutralPixelStageWithIntermediate(
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.approachIntermediateStagingFromBackdropTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralCenterSpikePickupPose,
                            posesForRouteStraight.intermediatePose))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.intermediatePose,
                            posesForRouteStraight.backdropStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionAndPark(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.additionalPixelScorePose))
                    .build();
            return pushPropScoreFive;
        }

        private Action PushTeamPropAndBackdropStage(Pose2d startPose, Pose2d spikePose, Pose2d scorePose) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .afterTime(1.6, ExtendLift(LiftStates.AUTO_LOW))
                    .splineToLinearHeading(scorePose, scorePose.heading.log(), fastVelocity, fastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        public Action ScorePixelAndNeutralPixelStageWithIntermediate(Pose2d scoreStaging, Pose2d intermediatePose, Pose2d neutralStagingPose, double neutralStagingApproachTangent) {
            Action scorePixelAndNeutralStageWithIntermediatePose = roadRunnerDrive.actionBuilder(scoreStaging)
                    .lineToX(scoreStaging.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .waitSeconds(.2)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.OPEN))
                    .waitSeconds(.2)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH))
                    .afterTime(.5, RetractLift())
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(intermediatePose), neutralStagingApproachTangent, superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(neutralStagingPose), neutralStagingApproachTangent,  superFastVelocity, superFastAcceleration)
                    .build();
            return scorePixelAndNeutralStageWithIntermediatePose;
        }


        public Action ScorePixelAndNeutralPixelStage(Pose2d scoreStaging, Pose2d neutralStagingPose, double neutralStagingApproachTangent) {
            Action scorePixelAndNeutralStageWithIntermediatePose = roadRunnerDrive.actionBuilder(scoreStaging)
                    .lineToX(scoreStaging.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .waitSeconds(.2)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.OPEN))
                    .waitSeconds(.4)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH))
                    .afterTime(.4, RetractLift())
                    .strafeToLinearHeading(PoseToVector(neutralStagingPose), neutralStagingApproachTangent,  superFastVelocity, superFastAcceleration)
                    .build();
            return scorePixelAndNeutralStageWithIntermediatePose;
        }
        public Action PickupPixels(Pose2d neutralPixelStagingPose, Pose2d neutralPickupPose, Pose2d intermediateStagingPose) {
            Action pickupPixels = roadRunnerDrive.actionBuilder(neutralPixelStagingPose)
                    .afterTime(.1, new ActuateGripperAction(GripperStates.OPEN))
                    .afterTime(.1, new TurnIntakeSlowReverse())
                    .setReversed(true)
                    .splineToLinearHeading(neutralPickupPose, TANGENT_TOWARD_AUDIENCE, slowVelocity, slowAcceleration)
                    .stopAndAdd(new TurnIntakeOn())
                    .waitSeconds(.2)
                    .splineToLinearHeading(intermediateStagingPose, TANGENT_TOWARD_BACKSTAGE, fastVelocity, fastAcceleration)
                    .build();
            return pickupPixels;
        }

        public Action PickupPixelsConstantHeading(Pose2d neutralPixelStagingPose, Pose2d neutralPickupPose, double approachPickupTangent, Pose2d intermediateStagingPose, double approachStagingTangent, Pose2d stagingScorePose, Pose2d scorePose, double approachScoreTangent) {
            Action pickupPixels = roadRunnerDrive.actionBuilder(neutralPixelStagingPose)
                    .afterTime(.1, new ActuateGripperAction(GripperStates.OPEN))
                    .afterTime(.1, new TurnIntakeSlowReverse())
                    .setReversed(true)
                    .splineToLinearHeading(neutralPickupPose, approachPickupTangent, slowVelocity, slowAcceleration)
                    .stopAndAdd(new TurnIntakeOn())
                    .waitSeconds(.2)
                    .setReversed(false)
                    .splineToConstantHeading(PoseToVector(intermediateStagingPose), approachStagingTangent, fastVelocity, slowAcceleration)
                    .afterTime(.1, new TurnIntakeReverse())
                    .afterTime(.1, new TurnIntakeOff())
                    .afterTime(1.2, new ActuateGripperAction(GripperStates.CLOSED))
                    .afterTime(1.4, ExtendLift(LiftStates.AUTO_MID))
                    .splineToConstantHeading(PoseToVector(stagingScorePose), Math.toRadians(0), superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(scorePose), approachScoreTangent)
                    .build();
            return pickupPixels;
        }

        public Action NeutralStagingToBackdropStagingWithIntermediate(Pose2d intermediatePose, Pose2d stagingScorePose, Pose2d scorePose, double approachScoreTangent) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(intermediatePose)
                    .afterTime(.1, new TurnIntakeReverse())
                    .afterTime(.1, new TurnIntakeOff())
                    .afterTime(1.2, new ActuateGripperAction(GripperStates.CLOSED))
                    .afterTime(1.4, ExtendLift(LiftStates.AUTO_MID))
                    .splineToConstantHeading(PoseToVector(stagingScorePose), Math.toRadians(0), superFastVelocity, superFastAcceleration)
                    .splineToConstantHeading(PoseToVector(scorePose), approachScoreTangent)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action ScorePixelActionAndPark(Pose2d scoreStaging, LiftStates scoreHeight, double scoreLeaveTangent, Pose2d parkPose) {
            Action scorePixelAndPark = roadRunnerDrive.actionBuilder(scoreStaging)
                    .lineToX(scoreStaging.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .waitSeconds(.2)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.OPEN))
                    .waitSeconds(.2)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH))
                    .afterTime(.5, RetractLift())
                    .setReversed(true)
                    .splineToLinearHeading(parkPose,scoreLeaveTangent, superFastVelocity, superFastAcceleration)
                    .build();
            return scorePixelAndPark;
        }

        private Action PushTeamPropAudienceAndGoToBackdrop(Pose2d startPose, Pose2d spikePose, Pose2d scorePose, Pose2d intermediatePose) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .splineToLinearHeading(intermediatePose, intermediatePose.heading.log(), fastVelocity, fastAcceleration)
                    .afterTime(2, ExtendLift(LiftStates.AUTO_LOW))
                    .splineToLinearHeading(scorePose, scorePose.heading.log(), fastVelocity, fastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        public Action PushPropScoreSix(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAudienceAndGoToBackdrop(posesForRouteStraight.startingPose, posesForRouteStraight.spikePose, posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.intermediatePose))
                    .stopAndAdd(new RouteBuilder().ScorePixelAndNeutralPixelStageWithIntermediate(
                            posesForRouteStraight.yellowPixelScorePose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.approachTrussStagingFromIntermediateTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.neutralTrussPickupPose,
                            posesForRouteStraight.intermediatePose
                            ))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.backdropStagingPose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent
                         ))
                    .stopAndAdd(new RouteBuilder().ScorePixelAndNeutralPixelStageWithIntermediate(
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.approachTrussPickupFromStagingTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralCenterSpikePickupPose,
                            posesForRouteStraight.intermediatePose
                            ))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.backdropStagingPose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent
                            ))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionAndPark(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.additionalPixelScorePose))
                    .build();
            return pushPropScoreFive;
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
                            new RotateShoulderAction(ShoulderStates.HALFWAY),
                            new ActuateGripperAction(GripperStates.CLOSED)
                    ),
                    new SleepAction(.2),
                    new MoveLiftSlideActionFinishImmediate(LiftStates.HOME),
                    new SleepAction(.8),
                    new RotateShoulderAction(ShoulderStates.INTAKE)
            );
        }

    }

}


