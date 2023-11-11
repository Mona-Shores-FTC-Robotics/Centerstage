package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_PARK_LANE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_PICKUP;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_STAGING;
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
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_PARK_LANE_D;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_PARK_LANE_F;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_PIXEL_PICKUP;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_STAGING;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.MeepMeepTesting;

public class PosesForRoute {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d firstPixelScorePose;
    public Pose2d additionalPixelScorePose;
    public Pose2d neutralStagingPose;
    public Pose2d neutralPickupPose;
    public Pose2d parkPose;
    public double parkOrientation;
    public Pose2d spikePose;

    PosesForRoute(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField, MeepMeepTesting.TeamPropLocation teamPropLocation){
        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(MeepMeepTesting.TeamPropLocation teamPropLocation, MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = BLUE_BACKDROP_LEFT;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = BLUE_BACKDROP_RIGHT;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        } else {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = RED_BACKDROP_LEFT;
                    if (sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_L;
                    } else spikePose = RED_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = RED_BACKDROP_RIGHT;
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
                    } else spikePose = RED_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(MeepMeepTesting.AllianceColor allianceColor) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE) {
            backdropStagingPose = BLUE_BACKDROP_STAGING;
            neutralStagingPose = BLUE_NEUTRAL_STAGING;
            neutralPickupPose = BLUE_NEUTRAL_PIXEL_PICKUP;
            additionalPixelScorePose = BLUE_BACKDROP_CENTER;
            parkPose = BLUE_BACKSTAGE_PARK_LANE_C;
            parkOrientation = FACE_45_DEGREES;

        } else {
            backdropStagingPose = RED_BACKDROP_STAGING;
            neutralStagingPose = RED_NEUTRAL_STAGING;
            neutralPickupPose = RED_NEUTRAL_PIXEL_PICKUP;
            additionalPixelScorePose = RED_BACKDROP_CENTER;
            parkPose = RED_BACKSTAGE_PARK_LANE_D;
            parkOrientation = FACE_315_DEGREES;
        }
    }

    private void SetStartingPose(MeepMeepTesting.AllianceColor allianceColor, MeepMeepTesting.SideOfField sideOfField) {
        if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.BLUE && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
        } else if (allianceColor == MeepMeepTesting.AllianceColor.RED && sideOfField == MeepMeepTesting.SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
        }
    }

}
