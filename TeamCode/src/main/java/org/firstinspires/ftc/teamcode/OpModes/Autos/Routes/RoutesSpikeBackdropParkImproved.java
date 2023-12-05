package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.ActuatePixelPusherAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem;

public class RoutesSpikeBackdropParkImproved {
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

    public static void BuildRoutes() {

        Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);

        //////////
        // LEFT //
        //////////

        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L_PAST, BLUE_BACKSTAGE_SPIKE_L_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L_DROP, BLUE_BACKSTAGE_SPIKE_L_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L_PAST, BLUE_AUDIENCE_SPIKE_L_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L_DROP, BLUE_AUDIENCE_SPIKE_L_PAST.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L_PAST, Math.toRadians(115))
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L_DROP, Math.toRadians(115))
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L_PAST, Math.toRadians(95))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L_DROP, Math.toRadians(95))
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .setTangent(TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        ///////////
        // RIGHT //
        ///////////

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R_PAST, RED_BACKSTAGE_SPIKE_R_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R_DROP, RED_BACKSTAGE_SPIKE_R_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R_PAST, Math.toRadians(55))
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R_DROP, Math.toRadians(55))
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();


        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R_PAST, BLUE_BACKSTAGE_SPIKE_R_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R_DROP, BLUE_BACKSTAGE_SPIKE_R_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R_PAST, BLUE_AUDIENCE_SPIKE_R_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R_DROP, BLUE_AUDIENCE_SPIKE_R_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .setTangent(TANGENT_TOWARD_RED)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();

        ////////////
        // CENTER //
        ////////////

        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C_PAST, BLUE_BACKSTAGE_SPIKE_C_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C_DROP, BLUE_BACKSTAGE_SPIKE_C_PAST.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C_PAST, BLUE_AUDIENCE_SPIKE_C_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C_DROP, BLUE_AUDIENCE_SPIKE_C_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C_PAST, RED_BACKSTAGE_SPIKE_C_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C_DROP, RED_BACKSTAGE_SPIKE_C_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C_PAST, RED_AUDIENCE_SPIKE_C_PAST.heading.log())
                .setReversed(true)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C_DROP, RED_AUDIENCE_SPIKE_C_DROP.heading.log())
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_CENTER, AUTO_HIGH, AUTO_MID))
                .build();
    }

    public static class ActionsForSpikeBackdrop {
        public Action ScoreAndBackup(Pose2d start_pose, LiftSlideSubsystem.LiftStates liftHeight) {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(liftHeight))
                    .waitSeconds(.5)
                    .lineToX(TILE*2+7)
                    .waitSeconds(.9)
                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.AUTO_MID))
                    .waitSeconds(.5)
                    .lineToX(TILE*2-5.5)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }

        public Action ScoreWithTwoHeightsAndBackup(Pose2d start_pose, LiftSlideSubsystem.LiftStates firstHeight, LiftSlideSubsystem.LiftStates secondHeight) {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(firstHeight))
                    .waitSeconds(.2)
                    .lineToX(TILE*2+7)
                    .waitSeconds(.2)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(secondHeight))
                    .waitSeconds(.2)
                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(firstHeight))
                    .waitSeconds(.3)
                    .lineToX(TILE*2-2)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }
    }
}
