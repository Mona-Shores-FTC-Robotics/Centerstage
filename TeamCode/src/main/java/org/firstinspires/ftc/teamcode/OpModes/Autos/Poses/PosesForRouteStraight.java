package org.firstinspires.ftc.teamcode.OpModes.Autos.Poses;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftStates.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.*;


import com.acmerobotics.roadrunner.Pose2d;

public class PosesForRouteStraight {
    public Pose2d startingPose;
    public Pose2d backdropStagingPose;
    public Pose2d yellowPixelScorePose;
    public double yellowPixelLeaveTangent;
    public Pose2d additionalPixelScorePose;
    public double additionalPixelScorePoseApproachTangent;
    public double additionalPixelScorePoseLeaveTangent;
    public Pose2d neutralStagingPose;
    public Pose2d neutralPickupPose;
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
                    additionalPixelScorePoseLeaveTangent = Math.toRadians(180);
                    if (sideOfField == SideOfField.AUDIENCE) {
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
                    if (sideOfField == SideOfField.AUDIENCE) {
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
                    yellowPixelLeaveTangent = Math.toRadians(180);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(15);
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(195);
                        spikePose = BLUE_AUDIENCE_SPIKE_C_DROP;
                    } else {
                        additionalPixelScorePose = BLUE_BACKDROP_LEFT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(195);
                        additionalPixelScorePoseApproachTangent = Math.toRadians(15);
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
                    if (sideOfField == SideOfField.AUDIENCE) {
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
                    if (sideOfField == SideOfField.AUDIENCE) {
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
                    yellowPixelLeaveTangent = Math.toRadians(180);
                    if (sideOfField == SideOfField.AUDIENCE) {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(-15);
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(165);
                        spikePose = RED_AUDIENCE_SPIKE_C_DROP;
                    } else
                    {
                        additionalPixelScorePoseApproachTangent = Math.toRadians(-15);
                        additionalPixelScorePose = RED_BACKDROP_RIGHT;
                        additionalPixelScorePoseLeaveTangent = Math.toRadians(165);
                        spikePose = RED_BACKSTAGE_SPIKE_C_DROP;
                    }
                    break;
                }
            }
        }
    }

    public void SetAlliancePoses(AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.BLUE) {
            backdropStagingPose = BLUE_BACKDROP_STAGING;
            neutralStagingPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = BLUE_MIDDLE_PARK;
            parkOrientation = FACE_45_DEGREES;

        } else {
            backdropStagingPose = RED_BACKDROP_STAGING;
            neutralStagingPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS;
            neutralPickupPose = SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP;
            parkPose = RED_MIDDLE_PARK;
            parkOrientation = FACE_315_DEGREES;
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
