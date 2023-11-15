package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

public class SpikeOnlyRoute {

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

    public static void BuildRoutes(MecanumDriveMona roadRunnerDriveSubsystem) {
        blueBackstageBotTeamPropLeftRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDriveSubsystem.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_POSE,TANGENT_TOWARD_RED)
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDriveSubsystem.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_START_POSE,TANGENT_TOWARD_RED)
                .build();

        blueBackstageBotTeamPropRightRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        blueAudienceBotTeamPropRightRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDriveSubsystem.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_POSE,TANGENT_TOWARD_RED)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDriveSubsystem.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_START_POSE,TANGENT_TOWARD_RED)
                .build();

        blueBackstageBotTeamPropCenterRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDriveSubsystem.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_START_POSE,TANGENT_TOWARD_BLUE)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDriveSubsystem.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_START_POSE,TANGENT_TOWARD_RED)
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDriveSubsystem.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
                .stopAndAdd(new ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates.CLOSED))
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_POSE,TANGENT_TOWARD_RED)
                .build();
    }

}
