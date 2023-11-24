package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.SideOfField.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.TeamPropLocation.*;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Poses.PosesForRouteSuper;

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
                .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStagingByWall(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingByWall(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper.parkPose))
                .build();
        return superBackstageAuto;
    }

    public Action SuperAudience(PosesForRouteSuper posesForRouteSuper) {
        Action superAudienceAuto = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.neutralStagingPose))
                .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper,posesForRouteSuper.additionalPixelScorePose))
                .stopAndAdd(new RouteBuilder().ScoreOnePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().BackdropStagingToNeutralStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.firstPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().PickupPixels(posesForRouteSuper, posesForRouteSuper.secondNeutralStagingPose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().NeutralStagingToBackdropStagingThroughStageDoor(posesForRouteSuper, posesForRouteSuper.additionalPixelScorePose))
//                .stopAndAdd(new RoutesSuper.RouteBuilder().ScorePixelAction(posesForRouteSuper.additionalPixelScorePose, posesForRouteSuper))
                .stopAndAdd(new RouteBuilder().Park(posesForRouteSuper.firstPixelScorePose, posesForRouteSuper.parkPose))
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
                            new TurnIntakeOn(),
                            new RouteBuilder().AutoDriveToNeutralStack(posesForRouteSuper, neutralPixelStagingPose)),
                    new SleepAction(.1),
                    new ParallelAction(
                            new TurnIntakeOff(),
                            new RouteBuilder().AutoDriveFromNeutralStack(posesForRouteSuper)));
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
                            new RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRouteSuper),
                            new MoveLiftSlideAction(AUTO_LOW),
                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP)),
                    new SleepAction(.2),
                    new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRouteSuper),
                            new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE)),
                    new MoveLiftSlideAction(HOME)
            );
            return scorePixel;
        }

        public Action ScoreOnePixelAction(Pose2d scorePose, PosesForRouteSuper posesForRouteSuper) {
            SequentialAction scorePixel = new SequentialAction(
                    new ParallelAction(
                            new RouteBuilder().AutoDriveToBackDrop(scorePose, posesForRouteSuper),
                            new MoveLiftSlideAction (AUTO_LOW),
                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP)),
                    new SleepAction(.2),
                    new ActuateGripperAction(GripperSubsystem.GripperStates.ONE_PIXEL_RELEASE_POSITION),
                    new SleepAction(.2),
                    new ParallelAction(
                            new RouteBuilder().AutoDriveFromBackDrop(scorePose, posesForRouteSuper),
                            new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE)),
                    new MoveLiftSlideAction(HOME)
            );
            return scorePixel;
        }

        private Action PushTeamPropAndBackdropStage(PosesForRouteSuper posesForRouteSuper) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteSuper.startingPose)
                    .splineToLinearHeading(posesForRouteSuper.spikePosePast, posesForRouteSuper.spikePosePast.heading.log())
                    .setReversed(true)
                    .splineToLinearHeading(posesForRouteSuper.spikePoseDrop, -posesForRouteSuper.spikePosePast.heading.log())
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
                    .splineToLinearHeading(posesForRouteSuper.spikePoseDrop, -posesForRouteSuper.spikePosePast.heading.log())
                    .setTangent(posesForRouteSuper.leaveSpikeTangent)
                    .setReversed(true)
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

