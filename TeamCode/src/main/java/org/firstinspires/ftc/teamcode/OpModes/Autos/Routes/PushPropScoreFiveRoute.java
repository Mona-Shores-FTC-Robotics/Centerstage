package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

//import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.*;
//import static org.firstinspires.ftc.teamcode.OpModes.Basic_Auto.roadRunnerDrive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_AUDIENCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.SideOfField.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.TeamPropLocation.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.LineToXRelativeAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOff;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions.TurnIntakeOn;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class PushPropScoreFiveRoute {
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

    public static Action moveAndReadyToScorePixel;
    public static Action releasePixelsAndMove;

    public static void BuildRoutes() {

        MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        /** BLUE BACKSTAGE RIGHT **/
        PosesForRoute blueBackstageRightPoses = new PosesForRoute(BLUE, BACKSTAGE, RIGHT);
        blueBackstageBotTeamPropRightRoute = Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder (blueBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageRightPoses))
                .build();

        /** RED BACKSTAGE RIGHT **/
        PosesForRoute redBackstageRightPoses = new PosesForRoute(RED, BACKSTAGE, RIGHT);
        redBackstageBotTeamPropRightRoute = mecanumDrive.actionBuilder(redBackstageRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageRightPoses))
                .build();

        /** BLUE AUDIENCE RIGHT **/
        PosesForRoute blueAudienceRightPoses = new PosesForRoute(BLUE, AUDIENCE, RIGHT);
        blueAudienceBotTeamPropRightRoute = mecanumDrive.actionBuilder(blueAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceRightPoses))
                .build();

        /** RED AUDIENCE RIGHT **/
        PosesForRoute redAudienceRightPoses = new PosesForRoute(RED, AUDIENCE, RIGHT);
        redAudienceBotTeamPropRightRoute = mecanumDrive.actionBuilder(redAudienceRightPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceRightPoses))
                .build();

        /** BLUE BACKSTAGE CENTER **/
        PosesForRoute blueBackstageCenterPoses = new PosesForRoute(BLUE, BACKSTAGE, CENTER);
        blueBackstageBotTeamPropCenterRoute = mecanumDrive.actionBuilder(blueBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageCenterPoses))
                .build();

        /** RED BACKSTAGE CENTER **/
        PosesForRoute redBackstageCenterPoses = new PosesForRoute(RED, BACKSTAGE, CENTER);
        redBackstageBotTeamPropCenterRoute = mecanumDrive.actionBuilder(redBackstageCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageCenterPoses))
                .build();

        /** BLUE AUDIENCE CENTER **/
        PosesForRoute blueAudienceCenterPoses = new PosesForRoute(BLUE, AUDIENCE, CENTER);
        blueAudienceBotTeamPropCenterRoute = mecanumDrive.actionBuilder(blueAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceCenterPoses))
                .build();

        /** RED AUDIENCE CENTER **/
        PosesForRoute redAudienceCenterPoses = new PosesForRoute(RED, AUDIENCE, CENTER);
        redAudienceBotTeamPropCenterRoute = mecanumDrive.actionBuilder(redAudienceCenterPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceCenterPoses))
                .build();

        /** BLUE BACKSTAGE LEFT **/
        PosesForRoute blueBackstageLeftPoses = new PosesForRoute(BLUE, BACKSTAGE, LEFT);
        blueBackstageBotTeamPropLeftRoute = mecanumDrive.actionBuilder(blueBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(blueBackstageLeftPoses))
                .build();

        /** RED BACKSTAGE LEFT **/
        PosesForRoute redBackstageLeftPoses = new PosesForRoute(RED, BACKSTAGE, LEFT);
        redBackstageBotTeamPropLeftRoute = mecanumDrive.actionBuilder(redBackstageLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreFive(redBackstageLeftPoses))
                .build();


        /** BLUE AUDIENCE LEFT **/
        PosesForRoute blueAudienceLeftPoses = new PosesForRoute(BLUE, AUDIENCE, LEFT);
        blueAudienceBotTeamPropLeftRoute = mecanumDrive.actionBuilder(blueAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(blueAudienceLeftPoses))
                .build();

        /** RED AUDIENCE LEFT **/
        PosesForRoute redAudienceLeftPoses = new PosesForRoute(RED, AUDIENCE, LEFT);
        redAudienceBotTeamPropLeftRoute = mecanumDrive.actionBuilder(redAudienceLeftPoses.startingPose)
                .stopAndAdd(new RouteBuilder().PushPropScoreSix(redAudienceLeftPoses))
                .build();

    }


    /**
     * METHODS TO SET SPIKE PIXEL ONLY ROUTES FOR ALL TEAM PROP LOCATIONS
     **/

    public static class RouteBuilder {
        MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        Action TrajectoryToBackDrop(Pose2d scorePose, PosesForRoute posesForRoute) {
            Action autoDriveToBackdrop = mecanumDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .splineToLinearHeading(scorePose, TANGENT_TOWARD_BACKSTAGE)
                    .build();
            return autoDriveToBackdrop;
        }

        Action AutoDriveFromBackDrop(Pose2d scorePose, PosesForRoute posesForRoute) {
            Action autoDriveFromBackdrop = mecanumDrive.actionBuilder(scorePose)
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRoute.backdropStagingPose), TANGENT_TOWARD_AUDIENCE)
                    .build();
            return autoDriveFromBackdrop;
        }

        public Action BackdropStagingToNeutralStaging(PosesForRoute posesForRoute) {
            Action backDropStagingToNeutralStaging = mecanumDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRoute.neutralStagingPose.position.x)
                    .build();
            return backDropStagingToNeutralStaging;
        }

        public Action NeutralStagingToBackdropStaging(PosesForRoute posesForRoute) {
            Action neutralStagingToBackdropStaging = mecanumDrive.actionBuilder(posesForRoute.neutralStagingPose)
                    .lineToX(posesForRoute.backdropStagingPose.position.x)
                    .build();
            return neutralStagingToBackdropStaging;
        }

        public Action PickupPixels(PosesForRoute posesForRoute) {
            SequentialAction pickupPixels = new SequentialAction(
                    new ParallelAction(
                            new TurnIntakeOn(),
                            new RouteBuilder().AutoDriveToNeutralStack(posesForRoute)),
                    new SleepAction(.1),
                    new ParallelAction(
                            new TurnIntakeOff(),
                            new RouteBuilder().AutoDriveFromNeutralStack(posesForRoute)));
            return pickupPixels;
        }

        private Action AutoDriveFromNeutralStack(PosesForRoute posesForRoute) {
            Action autoDriveFromNeutralStack = mecanumDrive.actionBuilder(posesForRoute.neutralPickupPose)
                    .lineToX(posesForRoute.neutralStagingPose.position.x)
                    .build();
            return autoDriveFromNeutralStack;
        }

        public Action AutoDriveToNeutralStack(PosesForRoute posesForRoute) {
            Action autoDriveToNeutralStack = mecanumDrive.actionBuilder(posesForRoute.neutralStagingPose)
                    .setReversed(true)
                    .lineToX(posesForRoute.neutralPickupPose.position.x)
                    .build();
            return autoDriveToNeutralStack;
        }



        //This action shouldn't require us to know we are as long as we have vision of an april tag
        //leaving the old "route" way of doing things here and the inputs it used for now incase the tag stuff doesnt work out
//        new RouteBuilder().TrajectoryToBackDrop(scorePose, posesForRoute)
//       new RouteBuilder().TrajectoryFromBackDrop(scorePose, posesForRoute)
        public Action ScorePixelAction(Pose2d scorePose, VisionSubsystem.DeliverLocation deliverLocation, PosesForRoute posesForRoute) {
            SequentialAction scorePixel =
                    new SequentialAction(
                            new ParallelAction(
                                    new SequentialAction(
                                            new AutoDriveToBackDrop(deliverLocation),
                                            new LineToXRelativeAction(+3)
                                    ),
                                    new SequentialAction(
                                            new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED),
                                            new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.SAFE),
                                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP),
                                            new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.SAFE)
                                    )),
                            new SleepAction(.9),
                            new SequentialAction(
                                    new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN),
                                    new SleepAction(.5),
                                    new LineToXRelativeAction(-5)
                            ),
                            new ParallelAction(
                                    new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.SAFE),
                                    new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED),
                                    new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY)
                            ),
                            new ParallelAction(
                                    new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE),
                                    new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.HOME)
                            )
            );
            return scorePixel;
        }



        private Action PushTeamPropAndBackdropStage(PosesForRoute posesForRoute) {
            Action pushTeamPropAndStage = mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.spikePose, posesForRoute.spikePose.heading.log())
                    .stopAndAdd(new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED))
                    .setReversed(true)
                    .splineToLinearHeading(posesForRoute.backdropStagingPose, posesForRoute.backdropStagingPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action PushTeamPropAndNeutralStage(PosesForRoute posesForRoute) {
            Action pushTeamPropAndStage = mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .splineToLinearHeading(posesForRoute.spikePose, posesForRoute.spikePose.heading.log())
                    .setReversed(true)
                    .splineToConstantHeading(PoseToVector(posesForRoute.neutralStagingPose), posesForRoute.neutralPickupPose.heading.log())
                    .turnTo(posesForRoute.neutralPickupPose.heading.log())
                    .build();
            return pushTeamPropAndStage;
        }

        private Action Park(PosesForRoute posesForRoute) {
            Action park = mecanumDrive.actionBuilder(posesForRoute.backdropStagingPose)
                    .strafeTo(PoseToVector(posesForRoute.parkPose))
                    .turnTo(posesForRoute.parkOrientation)
                    .build();
            return park;
        }

        public Action PushPropScoreFive(PosesForRoute posesForRoute) {
            Action pushPropScoreFive = mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndBackdropStage(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.firstPixelScorePose, posesForRoute.firstDeliverLocation, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute.additionalDeliverLocation, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute.additionalDeliverLocation, posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
            return pushPropScoreFive;
        }

        public Action PushPropScoreSix(PosesForRoute posesForRoute) {
            Action pushPropScoreFive = mecanumDrive.actionBuilder(posesForRoute.startingPose)
                    .stopAndAdd(new RouteBuilder().PushTeamPropAndNeutralStage(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute.firstDeliverLocation, posesForRoute))
                    .stopAndAdd(new RouteBuilder().BackdropStagingToNeutralStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().PickupPixels(posesForRoute))
                    .stopAndAdd(new RouteBuilder().NeutralStagingToBackdropStaging(posesForRoute))
                    .stopAndAdd(new RouteBuilder().ScorePixelAction(posesForRoute.additionalPixelScorePose, posesForRoute.additionalDeliverLocation, posesForRoute))
                    .stopAndAdd(new RouteBuilder().Park(posesForRoute))
                    .build();
            return pushPropScoreFive;
        }


    }
}

