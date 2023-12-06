package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.ActuatePixelPusherAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem;

import java.util.Arrays;

public class RoutesSpikeBackdropPark {

    public static double VELOCITY_OVERRIDE = 25;
    public static double ACCELERATION_OVERRIDE = 30;
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

    public static VelConstraint overrideVelConstraint;
    public static AccelConstraint overrideAccelConstraint;
    public static TurnConstraints overrideTurnConstraint;

    public static void BuildRoutes() {

        Action retractPusherToStopPushingPurplePixel = new ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates.NOT_PUSHING);

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(VELOCITY_OVERRIDE),
                        new AngularVelConstraint(VELOCITY_OVERRIDE)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(30), -Math.toRadians(TURN_OVERRIDE), Math.toRadians(TURN_OVERRIDE));


        MecanumDrive roadRunnerDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_LEFT, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_RIGHT, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_RIGHT, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_LEFT, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_CENTER, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_CENTER, LiftSlideSubsystem.LiftStates.AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(11)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_LEFT, LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_RIGHT, LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_RIGHT,LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE, overrideVelConstraint, overrideAccelConstraint) //WAS Redaudiencespike Right and tangent toward blue
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_LEFT, LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_CENTER, LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(retractPusherToStopPushingPurplePixel)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE, overrideVelConstraint, overrideAccelConstraint)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .waitSeconds(10)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE, overrideVelConstraint, overrideAccelConstraint)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_CENTER, LiftSlideSubsystem.LiftStates.AUTO_HIGH, LiftSlideSubsystem.LiftStates.AUTO_MID))
                .build();
    }

    public static class ActionsForSpikeBackdrop {
        public Action ScoreAndBackup(Pose2d start_pose, LiftSlideSubsystem.LiftStates liftHeight) {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(liftHeight))
                    .waitSeconds(.5)
                    .lineToX(start_pose.position.x+5.5, overrideVelConstraint, overrideAccelConstraint)
                    .waitSeconds(.9)
                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates.AUTO_MID))
                    .waitSeconds(.5)
                    .lineToX(start_pose.position.x)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }

        public Action ScoreWithTwoHeightsAndBackup(Pose2d start_pose, LiftSlideSubsystem.LiftStates firstHeight, LiftSlideSubsystem.LiftStates secondHeight) {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(firstHeight))
                    .waitSeconds(.2)
                    .lineToX(start_pose.position.x+5.5, overrideVelConstraint, overrideAccelConstraint)
                    .waitSeconds(.2)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(secondHeight))
                    .waitSeconds(.2)
                    .stopAndAdd( new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(firstHeight))
                    .waitSeconds(.3)
                    .lineToX(start_pose.position.x)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }

        public Action ForwardSixInches(Pose2d start_pose)
        {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .lineToX(start_pose.position.x+6)
                    .build();
        }


    }
}
