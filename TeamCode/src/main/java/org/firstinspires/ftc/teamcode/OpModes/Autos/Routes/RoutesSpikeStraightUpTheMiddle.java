package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.SideOfField.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.TeamPropLocation.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.ACCELERATION_OVERRIDE;
import static org.firstinspires.ftc.teamcode.OpModes.Autos.Routes.RoutesSpikeBackdropPark.VELOCITY_OVERRIDE;


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
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateGripperCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeReverse;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeSlowReverse;
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

    public static double SUPER_FAST_VELOCITY_OVERRIDE = 70;
    public static double SUPER_FAST_ACCELERATION_OVERRIDE = 90;
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

        Action AutoDriveFromBackDrop(Pose2d scorePose) {
            Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .lineToX(scorePose.position.x)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(Pose2d scorePose, Pose2d neutralStagingPose, double approachTangent) {
            Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(new Pose2d(scorePose.position.x+SCORE_DISTANCE, scorePose.position.y, scorePose.heading.log()))
                    .setReversed(true)
                    .setTangent(approachTangent)
                    .splineToLinearHeading(neutralStagingPose, TANGENT_TOWARD_AUDIENCE, superFastVelocity, superFastAcceleration)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(Pose2d startPose, Pose2d endPose, double approachTangent) {
            Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(startPose)
                    .setReversed(false)
                    .splineToConstantHeading(PoseToVector(endPose), approachTangent, superFastVelocity, superFastAcceleration)
                    .stopAndAdd(new ActuateGripperAction(GripperStates.CLOSED))
                    .stopAndAdd(new TurnIntakeOff())
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action PickupPixels(Pose2d neutralPixelStagingPose, Pose2d neutralPickupPose, double approachEndPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ActuateGripperAction(GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToNeutralStack(neutralPixelStagingPose, neutralPickupPose),
                    new TurnIntakeOn(),
                    new SleepAction(.1),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveFromNeutralStack(neutralPickupPose, neutralPixelStagingPose, approachEndPose));
            return pickupPixels;
        }

        public Action StrafeAndPickupPixelsFromStack(Pose2d neutralPixelStagingPose, Pose2d stackStagingPose, Pose2d stackPickupPose, double approachEndPose) {
            SequentialAction pickupPixels = new SequentialAction(
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().StrafeToTrussStack(neutralPixelStagingPose, stackStagingPose),
                    new ActuateGripperAction(GripperStates.OPEN),
                    new TurnIntakeSlowReverse(),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToNeutralStack(stackStagingPose, stackPickupPose),
                    new TurnIntakeOn(),
                    new SleepAction(.1),
                    new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveFromNeutralStack(stackPickupPose, neutralPixelStagingPose, approachEndPose));
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
//                    .splineToConstantHeading(PoseToVector(endPose), TANGENT_TOWARD_AUDIENCE)
                    .lineToX(endPose.position.x, slowVelocity, slowAcceleration)
                    .build();
            return autoDriveToNeutralStack;
        }

        public Action ScorePixelAction(Pose2d scoreStaging, LiftStates scoreHeight, double scoreLeaveTangent, Pose2d neutralStagingPose) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ParallelAction(
                            new ActuateGripperAction(GripperStates.CLOSED),
                                new RotateShoulderAction(ShoulderStates.BACKDROP)),
                            new SleepAction(.15),
                            new MoveLiftSlideActionFinishImmediate(scoreHeight),
                            new SleepAction(.15),
                            new RoutesSpikeStraightUpTheMiddle.RouteBuilder().AutoDriveToBackDrop(scoreStaging),
                            new SleepAction(.5),
                            new ActuateGripperAction(GripperStates.OPEN),
                            new SleepAction(.5),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.2),
                            new ParallelAction(
                                    new RouteBuilder().BackdropStagingToNeutralStaging(scoreStaging, neutralStagingPose, scoreLeaveTangent),
                                    new SequentialAction(
                                            new SleepAction(.7),
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
                            new ActuateGripperAction(GripperStates.CLOSED),
                            new SleepAction(.2),
                            new MoveLiftSlideActionFinishImmediate(LiftStates.AUTO_HIGH),
                            new SleepAction(.8),
                            new ParallelAction(
                                    new RoutesSuper.RouteBuilder().AutoDriveFromBackDrop(scorePose),
                                    new SequentialAction(
                                            new SleepAction(.9),
                                            new ParallelAction(
                                                    new RotateShoulderAction(ShoulderStates.HALFWAY),
                                                    new ActuateGripperAction(GripperStates.CLOSED)),
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
        public Action StrafeToTrussStack(Pose2d startPose, Pose2d endPose) {
            Action strafe = roadRunnerDrive.actionBuilder(startPose)
                    .strafeTo(PoseToVector(endPose))
                    .build();
            return strafe;
        }
        private Action PushTeamPropAndBackdropStage(Pose2d startPose, Pose2d spikePose, Pose2d scorePose) {
            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(startPose)
                    .splineToLinearHeading(spikePose, spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .setReversed(true)
                    .splineToLinearHeading(scorePose, scorePose.heading.log(), fastVelocity, fastAcceleration)
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRouteStraight posesForRouteStraight) {
            Action dropPurple = new ActuateGripperAction(GripperStates.CLOSED);

            Action pushTeamPropAndStage = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .splineToLinearHeading(posesForRouteStraight.spikePose, posesForRouteStraight.spikePose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(dropPurple)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRouteStraight.neutralTrussStagingPose), posesForRouteStraight.neutralTrussPickupPose.heading.log(), fastVelocity, fastAcceleration)
                    .stopAndAdd(new RotateShoulderAction(ShoulderStates.INTAKE))
                    .turnTo(posesForRouteStraight.neutralTrussPickupPose.heading.log())
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
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.yellowPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.offsetNeutralStagingPose))
                    .stopAndAdd(new RouteBuilder().StrafeAndPickupPixelsFromStack(posesForRouteStraight.offsetNeutralStagingPose, posesForRouteStraight.neutralTrussStagingPose, posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.approachOffsetNeutralStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.offsetNeutralStagingPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.additionalPixelScorePoseLeaveTangent, posesForRouteStraight.offsetNeutralStagingPose))
                    .stopAndAdd(new RouteBuilder().StrafeAndPickupPixelsFromStack(posesForRouteStraight.offsetNeutralStagingPose, posesForRouteStraight.neutralCenterSpikeStagingPose, posesForRouteStraight.neutralCenterSpikePickupPose, posesForRouteStraight.approachOffsetNeutralStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.offsetNeutralStagingPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.yellowPixelLeaveTangent, posesForRouteStraight.additionalPixelScorePose))
//                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRouteStraight posesForRouteStraight) {
            Action pushPropScoreFive = roadRunnerDrive.actionBuilder(posesForRouteStraight.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRouteStraight))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralTrussStagingPose, posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.approachTrussStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScoreOnePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight))
                    .stopAndAdd(new RouteBuilder().StrafeToPlaceFirstPixel(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.yellowPixelScorePose))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRouteStraight.yellowPixelScorePose, posesForRouteStraight.neutralTrussStagingPose, posesForRouteStraight.yellowPixelLeaveTangent))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRouteStraight.neutralTrussStagingPose, posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.approachTrussStagingTangent))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRouteStraight.neutralTrussPickupPose, posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelScorePoseApproachTangent))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRouteStraight.additionalPixelScorePose, posesForRouteStraight.additionalPixelPixelScoreHeight, posesForRouteStraight.additionalPixelScorePoseLeaveTangent, posesForRouteStraight.backdropStagingPose))
                    .stopAndAdd(new RouteBuilder().Park(posesForRouteStraight))
                    .build();
            return pushPropScoreFive;
        }
    }
}

