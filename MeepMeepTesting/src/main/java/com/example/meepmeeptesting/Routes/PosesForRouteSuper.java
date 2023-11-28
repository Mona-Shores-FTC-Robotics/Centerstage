package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R_PAST;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C_DROP;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C_PAST;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L_DROP;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L_PAST;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R_PAST;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C_DROP;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C_PAST;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L_DROP;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L_PAST;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R_PAST;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.RED_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_STAGEDOOR_BY_BACKDROP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BLUE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.MeepMeepTesting;

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
    public Pose2d spikePosePast;
    public Pose2d spikePoseDrop;
    public double startingTangent;


    PosesForRouteSuper(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField, MeepMeepTesting.TeamPropLocation teamPropLocation){
        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(MeepMeepTesting.TeamPropLocation teamPropLocation, MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = BLUE_BACKDROP_LEFT;
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePosePast = BLUE_AUDIENCE_SPIKE_L_PAST;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_L_DROP;
                        neutralApproachOrientation = Math.toRadians(-100);
                        leaveSpikeTangent = Math.toRadians(150);

                        startingTangent = TANGENT_TOWARD_RED;
                    } else {
                        spikePosePast = BLUE_BACKSTAGE_SPIKE_L_PAST;
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
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePosePast = BLUE_AUDIENCE_SPIKE_R_PAST;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_R_DROP;
                        neutralApproachOrientation = Math.toRadians(225);
                        leaveSpikeTangent = Math.toRadians(50);
                        startingTangent = Math.toRadians(-130);

                    } else {
                        spikePosePast = BLUE_BACKSTAGE_SPIKE_R_PAST;
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
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePose = BLUE_BACKDROP_RIGHT;
                        spikePosePast = BLUE_AUDIENCE_SPIKE_C_PAST;
                        spikePoseDrop = BLUE_AUDIENCE_SPIKE_C_DROP;
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_RED;
                    } else
                    {
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        spikePosePast = BLUE_BACKSTAGE_SPIKE_C_PAST;
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
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePosePast = RED_AUDIENCE_SPIKE_L_PAST;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_L_DROP;
                        neutralApproachOrientation = Math.toRadians(130);
                        leaveSpikeTangent = Math.toRadians(-50);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePosePast = RED_BACKSTAGE_SPIKE_L_PAST;
                        spikePoseDrop = RED_BACKSTAGE_SPIKE_L_DROP;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    }
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = RED_BACKDROP_RIGHT;
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePosePast = RED_AUDIENCE_SPIKE_R_PAST;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_R_DROP;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePosePast = RED_BACKSTAGE_SPIKE_R_PAST;
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
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePose = RED_BACKDROP_LEFT;
                        spikePosePast = RED_AUDIENCE_SPIKE_C_PAST;
                        spikePoseDrop = RED_AUDIENCE_SPIKE_C_DROP;
                        neutralApproachOrientation = Math.toRadians(45);
                        leaveSpikeTangent = Math.toRadians(180);
                        startingTangent = TANGENT_TOWARD_BLUE;
                    } else {
                        spikePosePast = RED_BACKSTAGE_SPIKE_C_PAST;
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

    public void SetAlliancePoses(MeepMeepTesting.AllianceColor allianceColor) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {

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

    private void SetStartingPose(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
            parkPose = BLUE_CORNER_PARK;
            leaveNeutralTangent = Math.toRadians(75);
            neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
            parkPose = BLUE_MIDDLE_PARK;
            leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
            neutralStagingPose = SUPER_BLUE_STAGEDOOR_ENTRANCE;
            neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
            parkPose = RED_CORNER_PARK;
            leaveNeutralTangent = Math.toRadians(-75);
            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
            parkPose = RED_MIDDLE_PARK;
            neutralStagingPose = SUPER_RED_STAGEDOOR_ENTRANCE;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
        }
    }

}
