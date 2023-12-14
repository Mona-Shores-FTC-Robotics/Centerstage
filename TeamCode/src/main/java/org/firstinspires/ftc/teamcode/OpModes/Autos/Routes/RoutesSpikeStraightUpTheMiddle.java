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

    public static double SUPER_FAST_VELOCITY_OVERRIDE = 60;
    public static double SUPER_FAST_ACCELERATION_OVERRIDE = 60;
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
        Action AutoDriveToBackDrop(Pose2d scorePose) {
            Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(scorePose)
                    .lineToX(scorePose.position.x+SCORE_DISTANCE, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveToBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(Pose2d scorePose, Pose2d neutralStagingPose, double approachTangent) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .setTangent(approachTangent)
                    .afterDisp(.5, RetractLift())
                    .splineToLinearHeading(neutralStagingPose, TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .build();
            return backDropStagingToNeutralStaging;
        }
        public Action BackdropStagingToNeutralStagingWithIntermediate(Pose2d scorePose, Pose2d neutralStagingPose, double scoreLeaveTangent, Pose2d intermediatePose, double neutralApproachTangent) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .setTangent(scoreLeaveTangent)
                    .afterDisp(.5, RetractLift())
                    .splineToLinearHeading(intermediatePose, TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .splineToLinearHeading(neutralStagingPose, neutralApproachTangent, superFastVelocity, superFastAcceleration)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStagingWithIntermediate(Pose2d startPose, Pose2d endPose, double approachTangent, Pose2d intermediatePose, double neutralLeaveTangent) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(false)
                    .setTangent(neutralLeaveTangent)
                    .splineToConstantHeading(PoseToVector(intermediatePose), approachTangent, superFastVelocity, superFastAcceleration)
                    .afterDisp(1.5, new ActuateGripperAction(GripperStates.CLOSED))
                    .afterDisp(1.6, new TurnIntakeOff())
                    .afterDisp(1.8, ExtendLift(LiftStates.AUTO_MID))
                    .splineToConstantHeading(PoseToVector(endPose), approachTangent, superFastVelocity, superFastAcceleration)
                    .build();
            return neutralStagingToBackdropStaging;
        }
        public Action PickupPixels(Pose2d neutralPixelStagingPose, Pose2d neutralPickupPose, double approachEndPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ActuateGripperAction(GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, neutralPickupPose),
                    new TurnIntakeOn(),
                    new SleepAction(.15),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveFromNeutralStack(neutralPickupPose, neutralPixelStagingPose, approachEndPose));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(Pose2d startPose, Pose2d endPose, double approachEndPose) {
            Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(false)
                    .splineToConstantHeading(PoseToVector(endPose), approachEndPose, slowVelocity, slowAcceleration)
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

        public Action ScorePixelAction(Pose2d scoreStaging, LiftStates scoreHeight, double scoreLeaveTangent, Pose2d neutralStagingPose) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToBackDrop(scoreStaging),
                            new SleepAction(.1),
                            new ActuateGripperAction(GripperStates.OPEN),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.2),
                            new RouteBuilder().BackdropStagingToNeutralStaging(scoreStaging, neutralStagingPose, scoreLeaveTangent)
                    );
            return scorePixel;
        }

        public Action ScorePixelActionWithIntermediatePose(Pose2d scoreStaging, double scoreLeaveTangent, Pose2d neutralStagingPose, Pose2d intermediatePose, double neutralApproachTangent) {
            SequentialAction scorePixel =
                                    new SequentialAction(
                            new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToBackDrop(scoreStaging),
                            new SleepAction(.1),
                            new ActuateGripperAction(GripperStates.OPEN),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.2),
                            new RouteBuilder().BackdropStagingToNeutralStagingWithIntermediate(scoreStaging, neutralStagingPose, scoreLeaveTangent, intermediatePose, neutralApproachTangent)
                    );
            return scorePixel;
        }

        private Action PushTeamPropAndBackdropStage(Pose2d startPose, Pose2d spikePose, Pose2d scorePose) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .afterDisp(1.6, ExtendLift(LiftStates.AUTO_LOW))
                    .splineToLinearHeading(scorePose, scorePose.heading.log(), fastVelocity, fastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAudienceAndGoToBackdrop(Pose2d startPose, Pose2d spikePose, Pose2d scorePose, Pose2d intermediatePose) {
            Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(retractPusherToStopPushingPurplePixel)
                    .setReversed(true)
                    .splineToLinearHeading(intermediatePose, intermediatePose.heading.log(), fastVelocity, fastAcceleration)
                    .afterDisp(3, ExtendLift(LiftStates.AUTO_LOW))
                    .splineToLinearHeading(scorePose, scorePose.heading.log(), fastVelocity, fastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        public Action PushPropScoreFive(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteStraight.startingPose, posesForRouteStraight.spikePose, posesForRouteStraight.yellowPixelScorePose))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionWithIntermediatePose(
                            posesForRouteStraight.yellowPixelScorePose,
                            posesForRouteStraight.yellowPixelLeaveTangent,
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            TANGENT_TOWARD_AUDIENCE))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.neutralTrussPickupPose,
                            posesForRouteStraight.approachTrussStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            TANGENT_TOWARD_BACKSTAGE))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionWithIntermediatePose(
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseLeaveTangent,
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralApproachTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralCenterSpikePickupPose,
                            TANGENT_TOWARD_BACKSTAGE))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralLeaveTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.additionalPixelScorePose))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAudienceAndGoToBackdrop(posesForRouteStraight.startingPose, posesForRouteStraight.spikePose, posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.intermediatePose))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionWithIntermediatePose(
                            posesForRouteStraight.yellowPixelScorePose,
                            posesForRouteStraight.yellowPixelLeaveTangent,
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            TANGENT_TOWARD_AUDIENCE))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralTrussStagingPose, posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.approachTrussStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralTrussStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            TANGENT_TOWARD_BACKSTAGE))
                    .stopAndAdd(new RouteBuilder().ScorePixelActionWithIntermediatePose(
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseLeaveTangent,
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralApproachTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.neutralCenterSpikePickupPose,
                            TANGENT_TOWARD_BACKSTAGE))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingWithIntermediate(
                            posesForRouteStraight.neutralCenterSpikeStagingPose,
                            posesForRouteStraight.additionalPixelScorePose,
                            posesForRouteStraight.additionalPixelScorePoseApproachTangent,
                            posesForRouteStraight.neutralPixelIntermediatePose,
                            posesForRouteStraight.neutralLeaveTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.additionalPixelScorePose))
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


