package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.Arrays;

public class RoutesSpikeBackdropPark {

    public static double VELOCITY_OVERRIDE = 15;
    public static double ACCELERATION_OVERRIDE = 15;
    public static double TURN_OVERRIDE=30;

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

    public static Action readyToScorePixel;
    public static Action releasePixels;

    public static VelConstraint overrideVelConstraint;
    public static AccelConstraint overrideAccelConstraint;
    public static TurnConstraints overrideTurnConstraint;

    public static void BuildRoutes() {

        Action dropPurple = new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED);

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(VELOCITY_OVERRIDE),
                        new AngularVelConstraint(VELOCITY_OVERRIDE)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(30), -Math.toRadians(TURN_OVERRIDE), Math.toRadians(TURN_OVERRIDE));


        MecanumDriveMona roadRunnerDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_L), TANGENT_TOWARD_RED)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE) //WAS Redaudiencespike Right and tangent toward blue
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction())
                .waitSeconds(.9)
                .stopAndAdd( new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.OPEN))
                .waitSeconds(.5)
                .lineToX(TILE*2-5.5)
                .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();
    }
}
