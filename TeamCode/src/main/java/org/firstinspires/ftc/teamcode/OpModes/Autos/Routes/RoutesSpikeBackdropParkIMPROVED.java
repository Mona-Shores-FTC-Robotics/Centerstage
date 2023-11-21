package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_SPIKE_R;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKDROP_CENTER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKDROP_LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKDROP_RIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_SPIKE_R;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_LANE_A;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_CORNER_PARK;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_PIXEL_STAGEDOOR;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_SAFE_STRAFE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_STAGEDOOR_ENTRANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_THROUGH_DOOR;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_135_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_225_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.FACE_TOWARD_BLUE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.PoseToVector;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_R;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_R_DROP;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_SPIKE_R_PAST;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_AUDIENCE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKDROP_CENTER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKDROP_LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKDROP_RIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_SPIKE_C;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_SPIKE_L;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_SPIKE_R;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_LANE_F;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_CORNER_PARK;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_NEUTRAL_PIXEL_STAGEDOOR;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_SAFE_STRAFE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_STAGEDOOR_ENTRANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_THROUGH_DOOR;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_135_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_225_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_315_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_45_DEGREES;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_BLUE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TANGENT_TOWARD_RED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.TILE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.AUTO_HIGH;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.AUTO_LOW;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.AUTO_MID;

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
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateEndEffectorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideActionFinishImmediate;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.Arrays;

public class RoutesSpikeBackdropParkIMPROVED {

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

        Action dropPurple = new ActuateEndEffectorAction(GripperSubsystem.GripperStates.CLOSED);

        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.new WheelVelConstraint(VELOCITY_OVERRIDE),
                        new AngularVelConstraint(VELOCITY_OVERRIDE)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

        overrideTurnConstraint = new TurnConstraints(
                Math.toRadians(TURN_OVERRIDE), -Math.toRadians(TURN_OVERRIDE), Math.toRadians(TURN_OVERRIDE));


        MecanumDriveMona roadRunnerDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        //////////
        // LEFT //
        //////////

        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(BLUE_BACKDROP_LEFT, AUTO_HIGH, AUTO_MID))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_LEFT, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE)
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
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .stopAndAdd(dropPurple)
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
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .waitSeconds(1)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreWithTwoHeightsAndBackup(RED_BACKDROP_RIGHT, AUTO_HIGH, AUTO_MID))
                .build();


        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_RIGHT, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropRightRoute  = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_L), TANGENT_TOWARD_RED)
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
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(BLUE_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(BLUE_CORNER_PARK))
                .build();

        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .stopAndAdd(dropPurple)
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
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_CENTER, TANGENT_TOWARD_BACKSTAGE)
                .stopAndAdd(new ActionsForSpikeBackdrop().ScoreAndBackup(RED_BACKDROP_CENTER, AUTO_LOW))
                .strafeTo(PoseToVector(RED_CORNER_PARK))
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .stopAndAdd(dropPurple)
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
                    .stopAndAdd( new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(AUTO_MID))
                    .waitSeconds(.5)
                    .lineToX(TILE*2-5.5)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }

        public Action ScoreWithTwoHeightsAndBackup(Pose2d start_pose, LiftSlideSubsystem.LiftStates firstHeight, LiftSlideSubsystem.LiftStates secondHeight) {
            return Robot.getInstance().getDriveSubsystem().mecanumDrive.actionBuilder(start_pose)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeReadyToScorePixelAction(firstHeight))
                    .waitSeconds(.2)
                    .lineToX(TILE*2+7, overrideVelConstraint, overrideAccelConstraint)
                    .waitSeconds(.2)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(secondHeight))
                    .waitSeconds(.2)
                    .stopAndAdd( new ActuateEndEffectorAction(GripperSubsystem.GripperStates.OPEN))
                    .waitSeconds(.5)
                    .stopAndAdd(new MoveLiftSlideActionFinishImmediate(firstHeight))
                    .waitSeconds(.3)
                    .lineToX(TILE*2-2)
                    .stopAndAdd(new MakeSpikeBackdropParkActions().MakeRetractArmAction())
                    .build();
        }
    }
}
