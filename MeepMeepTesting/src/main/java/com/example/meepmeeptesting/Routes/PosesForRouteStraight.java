package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.*;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R_DROP;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_MIDDLE_PARK;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.MeepMeepTesting;

public class PosesForRouteStraight {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d yellowPixelScorePose;
    public double yellowPixelLeaveTangent;
    public Pose2d additionalPixelScorePose;
    public double additionalPixelScorePoseApproachTangent;
    public double additionalPixelScorePoseLeaveTangent;
    public Pose2d offsetNeutralStagingPose;
    public double approachOffsetNeutralStagingTangent;
    public Pose2d neutralTrussStagingPose;
    public double approachTrussStagingTangent;
    public Pose2d neutralTrussPickupPose;
    public Pose2d neutralCenterSpikeStagingPose;
    public Pose2d neutralCenterSpikePickupPose;
    public double approacCenterSpikeStagingStagingTangent;
    public Pose2d parkPose;
    public double parkOrientation;
    public Pose2d spikePose;



    public MeepMeepTesting.LiftStates yellowPixelScoreHeight;
    public MeepMeepTesting.LiftStates additionalPixelPixelScoreHeight;

    PosesForRouteStraight(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField, MeepMeepTesting.TeamPropLocation teamPropLocation){


        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(MeepMeepTesting.TeamPropLocation teamPropLocation, MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    yellowPixelScorePose = BLUE_BACKDROP_LEFT;
                    yellowPixelLeaveTangent = Math.toRadians(195);
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = Math.toRadians(180);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_L_DROP;
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_BACKSTAGE_SPIKE_L_DROP;
                    }
                    break;
                }
                case RIGHT: {
                    yellowPixelScorePose = BLUE_BACKDROP_RIGHT;
                    yellowPixelLeaveTangent = Math.toRadians(165);
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = Math.toRadians(180);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_AUDIENCE_SPIKE_R_DROP;
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_BACKSTAGE_SPIKE_R_DROP;
                    }
                    break;
                }
                case CENTER:
                default: {
                    yellowPixelScorePose = BLUE_BACKDROP_CENTER;
                    yellowPixelLeaveTangent = Math.toRadians(175);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(15);
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(195);
                        spikePose = BLUE_AUDIENCE_SPIKE_C_DROP;
                    } else {
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(185);
                        additionalPixelScorePoseApproachTangent = Math.toRadians(5);
                        spikePose = BLUE_BACKSTAGE_SPIKE_C_DROP;
                    }
                    break;
                }
            }
        } else {
            switch (teamPropLocation) {
                case LEFT: {
                    yellowPixelScorePose = RED_BACKDROP_LEFT;
                    yellowPixelLeaveTangent = Math.toRadians(195);
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = Math.toRadians(180);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_AUDIENCE_SPIKE_L_DROP;
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_BACKSTAGE_SPIKE_L_DROP;
                    }
                    break;
                }
                case RIGHT: {
                    yellowPixelScorePose = RED_BACKDROP_RIGHT;
                    yellowPixelLeaveTangent = Math.toRadians(165);
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = Math.toRadians(180);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_AUDIENCE_SPIKE_R_DROP;
                    } else{
                        spikePose = RED_BACKSTAGE_SPIKE_R_DROP;
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                    }

                    break;
                }
                case CENTER:
                default: {
                    yellowPixelScorePose = RED_BACKDROP_CENTER;
                    yellowPixelLeaveTangent = Math.toRadians(185);
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(-10);
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(170);
                        spikePose = RED_AUDIENCE_SPIKE_C_DROP;
                    } else
                    {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(-10);
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(170);
                        spikePose = RED_BACKSTAGE_SPIKE_C_DROP;
                    }
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(MeepMeepTesting.AllianceColor allianceColor) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            offsetNeutralStagingPose= BLUE_OFFSET_NEUTRAL_PIXEL_STAGING;
            backdropStagingPose = BLUE_BACKDROP_STAGING;
            neutralTrussStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralTrussPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = BLUE_MIDDLE_PARK;
            parkOrientation = FACE_45_DEGREES;
            neutralCenterSpikeStagingPose = BLUE_NEUTRAL_PIXEL_CENTERSPIKE;
            neutralCenterSpikePickupPose = BLUE_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP;
            approachOffsetNeutralStagingTangent = TANGENT_TOWARD_BLUE;
            approachTrussStagingTangent = TANGENT_TOWARD_BACKSTAGE;
            approacCenterSpikeStagingStagingTangent = TANGENT_TOWARD_BACKSTAGE;

        } else {
            offsetNeutralStagingPose= RED_OFFSET_NEUTRAL_PIXEL_STAGING;
            backdropStagingPose = RED_BACKDROP_STAGING;
            neutralTrussStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
            neutralTrussPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = RED_MIDDLE_PARK;
            parkOrientation = FACE_315_DEGREES;
            neutralCenterSpikeStagingPose = RED_NEUTRAL_PIXEL_CENTERSPIKE;
            neutralCenterSpikePickupPose = RED_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP;
            approachOffsetNeutralStagingTangent = TANGENT_TOWARD_RED;
            approachTrussStagingTangent = TANGENT_TOWARD_BACKSTAGE;
            approacCenterSpikeStagingStagingTangent = TANGENT_TOWARD_BACKSTAGE;
        }
    }

    private void SetStartingPose(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {


        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
            yellowPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
            yellowPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
            yellowPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
            yellowPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = MeepMeepTesting.LiftStates.AUTO_MID;
        }
    }

}
