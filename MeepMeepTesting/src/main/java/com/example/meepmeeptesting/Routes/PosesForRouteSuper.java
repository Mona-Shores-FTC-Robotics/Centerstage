package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_EXIT;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.RED_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_EXIT;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_STAGEDOOR_BY_BACKDROP;

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
    public Pose2d secondNeutralStagingPose;
    public Pose2d secondNeutralPickupPose;
    public Pose2d audienceStartPose;
    public Pose2d backstageStartPose;
    public Pose2d parkPose;
    public double parkOrientation;
    public double backdropApproachOrientation;
    public double neutralApproachOrientation;
    public double leaveNeutralTangent;
    public Pose2d spikePose;

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
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = BLUE_BACKDROP_RIGHT;
                    additionalPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        additionalPixelScorePose = BLUE_BACKDROP_RIGHT;
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                    } else
                    {
                        spikePose = BLUE_BACKSTAGE_SPIKE_C;
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                    }
                    break;
                }
            }
        } else {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = RED_BACKDROP_LEFT;
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_L;
                    } else spikePose = RED_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = RED_BACKDROP_RIGHT;
                    additionalPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_R;
                    } else spikePose = RED_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_C;
                        additionalPixelScorePose = RED_BACKDROP_LEFT;

                    } else {
                        spikePose = RED_BACKSTAGE_SPIKE_C;
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                    }
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(MeepMeepTesting.AllianceColor allianceColor) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
            backdropStagingPose = BLUE_BACKDROP_STAGING;
            backstageStartPose = SUPER_BLUE_BACKSTAGE_START_POSE;
            audienceStartPose = SUPER_BLUE_AUDIENCE_START_POSE;
            backstageStageDoorPose = SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
            audienceStageDoorPose = BLUE_STAGEDOOR_ENTRANCE;
            parkOrientation = FACE_45_DEGREES;
            backdropApproachOrientation =Math.toRadians(0);
        } else {

            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
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
            neutralApproachOrientation =Math.toRadians(-105);

        } else if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
            parkPose = BLUE_MIDDLE_PARK;
            leaveNeutralTangent = Math.toRadians(-75);
            neutralApproachOrientation =Math.toRadians(105);
            secondNeutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
            secondNeutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;

        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
            parkPose = RED_CORNER_PARK;
            leaveNeutralTangent = Math.toRadians(-75);
            neutralApproachOrientation =Math.toRadians(105);
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
            parkPose = RED_MIDDLE_PARK;
            leaveNeutralTangent = Math.toRadians(75);
            neutralApproachOrientation =Math.toRadians(-105);
            secondNeutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
            secondNeutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
        }
    }

}
