package org.firstinspires.ftc.teamcode.OpModes.Autos.Routes;


import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_PIXEL_PICKUP;
import static com.example.meepmeeptesting.Constants.RED_NEUTRAL_STAGING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_PIXEL_PICKUP;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.BLUE_NEUTRAL_STAGING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.MeepMeepTesting;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

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

    PosesForRoute(InitVisionProcessor.AllianceColor allianceColor, InitVisionProcessor.SideOfField sideOfField, InitVisionProcessor.TeamPropLocation teamPropLocation){
        SetDeliverLocationPoses(teamPropLocation, allianceColor, sideOfField);
        SetStartingPose(allianceColor, sideOfField);
        SetAlliancePoses(allianceColor);
    }

    public void SetDeliverLocationPoses(InitVisionProcessor.TeamPropLocation teamPropLocation, InitVisionProcessor.AllianceColor allianceColor, InitVisionProcessor.SideOfField sideOfField) {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = BLUE_BACKDROP_LEFT;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_L;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = BLUE_BACKDROP_RIGHT;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_R;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = BLUE_BACKDROP_CENTER;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = BLUE_AUDIENCE_SPIKE_C;
                    } else spikePose = BLUE_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        } else {
            switch (teamPropLocation) {
                case LEFT: {
                    firstPixelScorePose = RED_BACKDROP_LEFT;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_L;
                    } else spikePose = RED_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    firstPixelScorePose = RED_BACKDROP_RIGHT;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_R;
                    } else spikePose = RED_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    firstPixelScorePose = RED_BACKDROP_CENTER;
                    if (sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
                        spikePose = RED_AUDIENCE_SPIKE_C;
                    } else spikePose = RED_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(InitVisionProcessor.AllianceColor allianceColor) {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE) {
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

    private void SetStartingPose(InitVisionProcessor.AllianceColor allianceColor, InitVisionProcessor.SideOfField sideOfField) {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
        }
    }

}
