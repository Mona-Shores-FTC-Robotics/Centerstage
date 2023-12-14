package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.*;
import static com.example.meepmeeptesting.MeepMeepTesting.*;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
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

public class PosesForRouteStraight {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d yellowPixelScorePose;
    public double yellowPixelLeaveTangent;

    public Pose2d additionalPixelStagingPose;
    public Pose2d additionalPixelScorePose;
    public double additionalPixelScorePoseApproachTangent;
    public double additionalPixelScorePoseLeaveTangent;


    public Pose2d neutralTrussStagingPose;
    public Pose2d neutralTrussPickupPose;
    public double approachTrussPickupFromStagingTangent;
    public double approachIntermediateStagingFromPickupTangent;
    public double approachIntermediateStagingFromBackdropTangent;
    public double approachTrussStagingFromIntermediateTangent;

    public Pose2d neutralCenterSpikeStagingPose;
    public Pose2d neutralCenterSpikePickupPose;
    public double approachCenterSpikeStagingFromPickupTangent;
    public double approachCenterSpikePickupFromStagingTangent;

    public Pose2d intermediatePose;
    public double intermediateTangent;
    public Pose2d neutralPixelIntermediatePose;
    public double neutralLeaveTangentFromPickup;

    public Pose2d parkPose;
    public double parkOrientation;
    public Pose2d spikePose;



    public LiftStates yellowPixelScoreHeight;
    public LiftStates additionalPixelPixelScoreHeight;

    public PosesForRouteStraight(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation){


        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(TeamPropLocation teamPropLocation, AllianceColor allianceColor, SideOfField sideOfField) {
        if (allianceColor == AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    yellowPixelScorePose = BLUE_BACKDROP_LEFT;
                    yellowPixelLeaveTangent = Math.toRadians(195);
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;
                    approachTrussStagingFromIntermediateTangent = Math.toRadians(172);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_BACKSTAGE_SPIKE_L;
                    }
                    break;
                }
                case RIGHT: {
                    yellowPixelScorePose = BLUE_BACKDROP_RIGHT;
                    yellowPixelLeaveTangent = Math.toRadians(165);
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;
                    approachTrussStagingFromIntermediateTangent = Math.toRadians(188);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_BACKSTAGE_SPIKE_R;
                    }
                    break;
                }
                case CENTER:
                default: {
                    yellowPixelScorePose = BLUE_BACKDROP_CENTER;
                    yellowPixelLeaveTangent = Math.toRadians(175);
                    approachTrussStagingFromIntermediateTangent = Math.toRadians(180);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        additionalPixelScorePose = SUPER_BLUE_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                    } else {
                        additionalPixelScorePose = SUPER_BLUE_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = BLUE_BACKSTAGE_SPIKE_C;
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
                    approachTrussStagingFromIntermediateTangent = Math.toRadians(172);
                    additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_AUDIENCE_SPIKE_L;
                    } else {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_BACKSTAGE_SPIKE_L;
                    }
                    break;
                }
                case RIGHT: {
                    yellowPixelScorePose = RED_BACKDROP_RIGHT;
                    yellowPixelLeaveTangent = Math.toRadians(175);
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    additionalPixelScorePoseLeaveTangent = TANGENT_TOWARD_AUDIENCE;

                    approachTrussStagingFromIntermediateTangent = Math.toRadians(188);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        spikePose = RED_AUDIENCE_SPIKE_R;
                    } else{
                        spikePose = RED_BACKSTAGE_SPIKE_R;
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                    }

                    break;
                }
                case CENTER:
                default: {
                    yellowPixelScorePose = RED_BACKDROP_CENTER;
                    yellowPixelLeaveTangent = Math.toRadians(180);

                    approachTrussStagingFromIntermediateTangent = Math.toRadians(180);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        additionalPixelScorePose = SUPER_RED_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(175);
                        spikePose = RED_AUDIENCE_SPIKE_C;
                    } else
                    {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(0);
                        additionalPixelScorePose = SUPER_RED_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(175);
                        spikePose = RED_BACKSTAGE_SPIKE_C;
                    }
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.BLUE) {
            backdropStagingPose = BLUE_BACKDROP_STAGING;
            neutralTrussStagingPose = BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralTrussPickupPose = BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = BLUE_MIDDLE_PARK;
            parkOrientation = FACE_45_DEGREES;
            neutralCenterSpikeStagingPose = BLUE_NEUTRAL_PIXEL_CENTERSPIKE;
            neutralCenterSpikePickupPose = BLUE_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP;
            approachTrussPickupFromStagingTangent = TANGENT_TOWARD_AUDIENCE;
            approachIntermediateStagingFromPickupTangent = TANGENT_TOWARD_BACKSTAGE;
            approachCenterSpikeStagingFromPickupTangent = TANGENT_TOWARD_BACKSTAGE;
            approachCenterSpikePickupFromStagingTangent = Math.toRadians(15);
            neutralPixelIntermediatePose = BLUE_MIDDLE_OF_SPIKES;
            neutralLeaveTangentFromPickup = Math.toRadians(30);
            intermediatePose = BLUE_MIDDLE_OF_SPIKES;
            intermediateTangent = FACE_TOWARD_BACKSTAGE;
            approachIntermediateStagingFromBackdropTangent= TANGENT_TOWARD_AUDIENCE;

        } else {
            backdropStagingPose = RED_BACKDROP_STAGING;
            neutralTrussStagingPose = RED_NEUTRAL_PIXEL_TRUSS;
            neutralTrussPickupPose = RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = RED_MIDDLE_PARK;
            parkOrientation = FACE_315_DEGREES;
            neutralCenterSpikeStagingPose = RED_NEUTRAL_PIXEL_CENTERSPIKE;
            neutralCenterSpikePickupPose = RED_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP;
            approachIntermediateStagingFromBackdropTangent= TANGENT_TOWARD_AUDIENCE;
            approachTrussPickupFromStagingTangent = TANGENT_TOWARD_AUDIENCE;
            approachIntermediateStagingFromPickupTangent = TANGENT_TOWARD_BACKSTAGE;
            approachCenterSpikeStagingFromPickupTangent = Math.toRadians(-15);
            approachCenterSpikePickupFromStagingTangent = Math.toRadians(-15);
            neutralPixelIntermediatePose = RED_MIDDLE_OF_SPIKES;
            neutralLeaveTangentFromPickup =  Math.toRadians(-15);

            intermediatePose = RED_MIDDLE_OF_SPIKES;;
            intermediateTangent = FACE_TOWARD_BACKSTAGE;

        }
    }

    private void SetStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {


        if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
            yellowPixelScoreHeight = LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
            yellowPixelScoreHeight = LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
            yellowPixelScoreHeight = LiftStates.AUTO_LOW;
            additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
            yellowPixelScoreHeight = LiftStates.AUTO_MID;
            additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
        }
    }

}

