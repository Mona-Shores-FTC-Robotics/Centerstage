package org.firstinspires.ftc.teamcode.OpModes.Autos.Poses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.*;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

public class PosesForRouteSuper {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d backstageStageDoorPose;
    public Pose2d audienceStageDoorPose;
    public Pose2d firstPixelScorePose;
    public Pose2d additionalPixelScorePose;
    public Pose2d neutralStagingPose;

    public Pose2d neutralPickupPose;
    public Pose2d audienceStartPose;
    public Pose2d backstageStartPose;
    public Pose2d parkPose;
    public double parkOrientation;
    public double backdropApproachOrientation;
    public double neutralApproachOrientation;
    public double leaveNeutralTangent;
    public double leaveSpikeTangent;
    public Pose2d spikePose;
    public Pose2d spikePoseDrop;
    public double startingTangent;
    public LiftSlideSubsystem.LiftStates firstPixelScoreHeight;
    public  LiftSlideSubsystem.LiftStates  additionalPixelPixelScoreHeight;


    public PosesForRouteSuper(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation){
        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(TeamPropLocation teamPropLocation, AllianceColor allianceColor, SideOfField sideOfField) {
        if (allianceColor == AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = BLUE_BACKDROP_LEFT;
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_L_DROP;
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        leaveSpikeTangent = Math.toRadians(0);
                        startingTangent = TANGENT_TOWARD_RED;
                    } else {
                        spikePose = BLUE_BACKSTAGE_SPIKE_L;
                        spikePoseDrop = BLUE_BACKSTAGE_SPIKE_L_DROP;
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_RED;
                    }
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = BLUE_BACKDROP_RIGHT;
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_R_DROP;
                        neutralApproachOrientation = Math.toRadians(0);
                        leaveSpikeTangent = Math.toRadians(0);
                        startingTangent = Math.toRadians(-130);

                    } else {
                        spikePose = BLUE_BACKSTAGE_SPIKE_R;
                        spikePoseDrop = BLUE_BACKSTAGE_SPIKE_R_DROP;
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_RED;
                    }
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePose = BLUE_BACKDROP_RIGHT;
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_C_DROP;
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_RED;
                    } else
                    {
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        spikePose = BLUE_BACKSTAGE_SPIKE_C;
                        spikePoseDrop = BLUE_BACKSTAGE_SPIKE_C_DROP;
                        neutralApproachOrientation =Math.toRadians(-105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_RED;
                    }
                    break;
                }
            }
        } else {
            ///////////////////////////////////
            //RED RED RED RED RED RED RED RED//
            ///////////////////////////////////
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = RED_BACKDROP_LEFT;
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_L;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_L_DROP;
                        neutralApproachOrientation = Math.toRadians(75);
                        leaveSpikeTangent = Math.toRadians(0);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePose = RED_BACKSTAGE_SPIKE_L;
                        spikePoseDrop = RED_BACKSTAGE_SPIKE_L;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    }
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = RED_BACKDROP_RIGHT;
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_R;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_R_DROP;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePose = RED_BACKSTAGE_SPIKE_R;
                        spikePoseDrop = RED_BACKSTAGE_SPIKE_R_DROP;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    }
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePose = RED_BACKDROP_LEFT;
                        spikePose = RED_AUDIENCE_SPIKE_C;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_C_DROP;
                        neutralApproachOrientation = Math.toRadians(45);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePose = RED_BACKSTAGE_SPIKE_C;
                        spikePoseDrop = RED_BACKSTAGE_SPIKE_C_DROP;
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    }
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.BLUE) {

            backdropStagingPose = BLUE_BACKDROP_STAGING;
            backstageStartPose = SUPER_BLUE_BACKSTAGE_START_POSE;
            audienceStartPose = SUPER_BLUE_AUDIENCE_START_POSE;
            backstageStageDoorPose = SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
            audienceStageDoorPose = BLUE_STAGEDOOR_ENTRANCE;
            parkOrientation = FACE_45_DEGREES;
            backdropApproachOrientation =Math.toRadians(0);
        } else {
            backstageStageDoorPose = SUPER_RED_STAGEDOOR_BY_BACKDROP;
            audienceStageDoorPose = RED_STAGEDOOR_ENTRANCE;
            backdropStagingPose = RED_BACKDROP_STAGING;
            backstageStartPose = SUPER_RED_BACKSTAGE_START_POSE;
            audienceStartPose = SUPER_RED_AUDIENCE_START_POSE;
            parkOrientation = FACE_315_DEGREES;
            backdropApproachOrientation = Math.toRadians(0);
        }
    }

    private void SetStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {
        if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
            parkPose = BLUE_CORNER_PARK;
            leaveNeutralTangent = Math.toRadians(75);
            neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
            firstPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.AUDIENCE) {
            startingPose = BLUE_NEUTRAL_STAGING;
            parkPose = BLUE_MIDDLE_PARK;
            leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
            firstPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
            parkPose = RED_CORNER_PARK;
            leaveNeutralTangent = Math.toRadians(-75);
            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
            firstPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
            parkPose = RED_MIDDLE_PARK;
            leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
            firstPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = LiftSlideSubsystem.LiftStates.AUTO_MID;
        }
    }

}
