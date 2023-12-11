package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_C;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_L;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_SPIKE_R;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
import static com.example.meepmeeptesting.Constants.SUPER_RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.MeepMeepTesting.*;
import static com.example.meepmeeptesting.Constants.BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.BLUE_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.BLUE_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.FACE_315_DEGREES;
import static com.example.meepmeeptesting.Constants.FACE_45_DEGREES;
import static com.example.meepmeeptesting.Constants.RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.RED_CORNER_PARK;
import static com.example.meepmeeptesting.Constants.RED_MIDDLE_PARK;
import static com.example.meepmeeptesting.Constants.RED_STAGEDOOR_ENTRANCE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_AUDIENCE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_BACKSTAGE_START_POSE;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS;
import static com.example.meepmeeptesting.Constants.SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
import static com.example.meepmeeptesting.Constants.SUPER_RED_STAGEDOOR_BY_BACKDROP;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_RED;

import com.acmerobotics.roadrunner.Pose2d;

public class PosesForRouteSuper {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d backstageStageDoorPose;
    public Pose2d audienceStageDoorPose;
    public Pose2d yellowPixelScorePose;
    public Pose2d additionalWhitePixelScorePose;
    public Pose2d neutralStagingPose;
    public Pose2d neutralPickupPose;
    public Pose2d audiencePathPose;
    public Pose2d backstagePathPose;
    public Pose2d parkPose;
    public double parkOrientation;
    public double backdropApproachOrientation;
    public double neutralApproachOrientation;
    public double leaveNeutralTangent;
    public double leaveSpikeTangent;
    public Pose2d spikePose;
    public LiftStates yellowPixelScoreHeight;
    public LiftStates  additionalPixelPixelScoreHeight;


    public PosesForRouteSuper(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation){
        ///////////////////////////////////
        //BLUE BLUE BLUE BLUE BLUE BLUE  //
        ///////////////////////////////////
        if (allianceColor == AllianceColor.BLUE) {
            backstagePathPose = SUPER_BLUE_BACKSTAGE_START_POSE;
            audiencePathPose = SUPER_BLUE_AUDIENCE_START_POSE;
            backstageStageDoorPose = SUPER_BLUE_STAGEDOOR_BY_BACKDROP;
            audienceStageDoorPose = BLUE_STAGEDOOR_ENTRANCE;

            parkOrientation = FACE_45_DEGREES;
            backdropApproachOrientation =Math.toRadians(0);

            switch (teamPropLocation) {
                ///////////////////////////////////
                //BLUE LEFT BLUE LEFT BLUE LEFT  //
                ///////////////////////////////////
                case LEFT: {
                    yellowPixelScorePose = BLUE_BACKDROP_LEFT;
                    additionalWhitePixelScorePose = BLUE_BACKDROP_RIGHT;

                    //////////////////////////////////////////
                    //BLUE LEFT AUDIENCE BLUE LEFT AUDIENCE //
                    //////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = BLUE_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                        leaveSpikeTangent = Math.toRadians(135);
                        neutralApproachOrientation = TANGENT_TOWARD_RED;
                        neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        parkPose = BLUE_MIDDLE_PARK;
                    }
                    ////////////////////////////////////////////
                    //BLUE LEFT BACKSTAGE BLUE LEFT BACKSTAGE //
                    ////////////////////////////////////////////
                    else {
                        startingPose = BLUE_BACKSTAGE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = BLUE_BACKSTAGE_SPIKE_L;
                        neutralApproachOrientation = Math.toRadians(-105);
                        leaveSpikeTangent = Math.toRadians(180);
                        leaveNeutralTangent = Math.toRadians(75);
                        neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
                        parkPose = BLUE_CORNER_PARK;
                    }
                    break;
                }
                /////////////////////////////////////
                //BLUE RIGHT BLUE RIGHT BLUE RIGHT //
                /////////////////////////////////////
                case RIGHT: {
                    yellowPixelScorePose = BLUE_BACKDROP_RIGHT;
                    additionalWhitePixelScorePose = BLUE_BACKDROP_CENTER;

                    ////////////////////////////////////////////
                    //BLUE RIGHT AUDIENCE BLUE RIGHT AUDIENCE //
                    ////////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = BLUE_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                        leaveSpikeTangent = Math.toRadians(45);
                        neutralApproachOrientation = Math.toRadians(-165);
                        neutralStagingPose = SUPER_BLUE_STAGEDOOR_ENTRANCE;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        parkPose = BLUE_MIDDLE_PARK;
                    }
                    //////////////////////////////////////////////
                    //BLUE RIGHT BACKSTAGE BLUE RIGHT BACKSTAGE //
                    //////////////////////////////////////////////
                    else {
                        startingPose = BLUE_BACKSTAGE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = BLUE_BACKSTAGE_SPIKE_R;
                        neutralApproachOrientation = Math.toRadians(-105);
                        leaveSpikeTangent = Math.toRadians(180);
                        leaveNeutralTangent = Math.toRadians(75);
                        neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
                        parkPose = BLUE_CORNER_PARK;
                    }
                    break;
                }
                ////////////////////////////////////////
                //BLUE CENTER BLUE CENTER BLUE CENTER //
                ////////////////////////////////////////
                case CENTER:
                default: {
                    yellowPixelScorePose = BLUE_BACKDROP_CENTER;

                    //////////////////////////////////////////////
                    //BLUE CENTER AUDIENCE BLUE CENTER AUDIENCE //
                    //////////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = BLUE_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalWhitePixelScorePose = BLUE_BACKDROP_RIGHT;
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                        leaveSpikeTangent = Math.toRadians(135);
                        neutralApproachOrientation = Math.toRadians(-105);
                        neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        parkPose = BLUE_MIDDLE_PARK;
                    }
                    ////////////////////////////////////////////////
                    //BLUE CENTER BACKSTAGE BLUE CENTER BACKSTAGE //
                    ////////////////////////////////////////////////
                    else
                    {
                        startingPose = BLUE_BACKSTAGE_START_POSE;
                        parkPose = BLUE_CORNER_PARK;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalWhitePixelScorePose = BLUE_BACKDROP_LEFT;
                        spikePose = BLUE_BACKSTAGE_SPIKE_C;
                        neutralApproachOrientation =Math.toRadians(-105);
                        leaveSpikeTangent = Math.toRadians(180);
                        leaveNeutralTangent = Math.toRadians(75);
                        neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
                    }
                    break;
                }
            }
        } else {
            ///////////////////////////////////
            //RED RED RED RED RED RED RED RED//
            ///////////////////////////////////
            backstageStageDoorPose = SUPER_RED_STAGEDOOR_BY_BACKDROP;
            audienceStageDoorPose = RED_STAGEDOOR_ENTRANCE;
            backdropStagingPose = RED_BACKDROP_STAGING;
            backstagePathPose = SUPER_RED_BACKSTAGE_START_POSE;
            audiencePathPose = SUPER_RED_AUDIENCE_START_POSE;
            parkOrientation = FACE_315_DEGREES;
            backdropApproachOrientation = Math.toRadians(0);

            switch (teamPropLocation) {
                ////////////////////////////////
                //RED LEFT RED LEFT RED LEFT  //
                ////////////////////////////////
                case LEFT: {
                    yellowPixelScorePose = RED_BACKDROP_LEFT;
                    additionalWhitePixelScorePose = RED_BACKDROP_CENTER;

                    ////////////////////////////////////////
                    //RED LEFT AUDIENCE RED LEFT AUDIENCE //
                    ////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = RED_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = RED_AUDIENCE_SPIKE_L;
                        leaveSpikeTangent = Math.toRadians(-45);
                        neutralApproachOrientation = Math.toRadians(165);
                        neutralStagingPose = SUPER_RED_STAGEDOOR_ENTRANCE;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        parkPose = RED_MIDDLE_PARK;
                    }
                    //////////////////////////////////////////
                    //RED LEFT BACKSTAGE RED LEFT BACKSTAGE //
                    //////////////////////////////////////////
                    else {
                        startingPose = RED_BACKSTAGE_START_POSE;
                        parkPose = RED_CORNER_PARK;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = RED_BACKSTAGE_SPIKE_L;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        leaveNeutralTangent = Math.toRadians(-75);
                        neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
                    }
                    break;
                }
                ///////////////////////////////////
                //RED RIGHT RED RIGHT RED RIGHT  //
                ///////////////////////////////////
                case RIGHT: {
                    yellowPixelScorePose = RED_BACKDROP_RIGHT;
                    additionalWhitePixelScorePose = RED_BACKDROP_CENTER;

                    //////////////////////////////////////////
                    //RED RIGHT AUDIENCE RED RIGHT AUDIENCE //
                    //////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = RED_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = RED_AUDIENCE_SPIKE_R;
                        leaveSpikeTangent = Math.toRadians(215);
                        neutralApproachOrientation = Math.toRadians(105);

                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        parkPose = RED_MIDDLE_PARK;
                    }
                    ////////////////////////////////////////////
                    //RED RIGHT BACKSTAGE RED RIGHT BACKSTAGE //
                    ////////////////////////////////////////////
                    else {
                        startingPose = RED_BACKSTAGE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = RED_BACKSTAGE_SPIKE_R;
                        leaveSpikeTangent = Math.toRadians(180);
                        neutralApproachOrientation = Math.toRadians(105);
                        neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
                        leaveNeutralTangent = Math.toRadians(-75);
                        parkPose = RED_CORNER_PARK;
                    }
                    break;
                }
                //////////////////////////////////////
                //RED CENTER RED CENTER RED CENTER  //
                //////////////////////////////////////
                case CENTER:
                default: {
                    yellowPixelScorePose = RED_BACKDROP_CENTER;

                    ////////////////////////////////////////////
                    //RED CENTER AUDIENCE RED CENTER AUDIENCE //
                    ////////////////////////////////////////////
                    if (sideOfField == SideOfField.AUDIENCE) {
                        startingPose = RED_AUDIENCE_START_POSE;
                        yellowPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        additionalWhitePixelScorePose = RED_BACKDROP_LEFT;
                        spikePose = RED_AUDIENCE_SPIKE_C;
                        leaveSpikeTangent = Math.toRadians(-135);
                        neutralApproachOrientation = Math.toRadians(105);
                        neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP;
                        leaveNeutralTangent = TANGENT_TOWARD_BACKSTAGE;
                        parkPose = RED_MIDDLE_PARK;
                    }
                    //////////////////////////////////////////////
                    //RED CENTER BACKSTAGE RED CENTER BACKSTAGE //
                    //////////////////////////////////////////////
                    else {
                        startingPose = RED_BACKSTAGE_START_POSE;
                        parkPose = RED_CORNER_PARK;
                        yellowPixelScoreHeight = LiftStates.AUTO_LOW;
                        additionalPixelPixelScoreHeight = LiftStates.AUTO_MID;
                        spikePose = RED_BACKSTAGE_SPIKE_C;
                        additionalWhitePixelScorePose = RED_BACKDROP_RIGHT;
                        neutralApproachOrientation = Math.toRadians(105);
                        leaveSpikeTangent = Math.toRadians(180);
                        leaveNeutralTangent = Math.toRadians(-75);
                        neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
                        neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
                    }
                    break;
                }
            }
        }
    }

}
