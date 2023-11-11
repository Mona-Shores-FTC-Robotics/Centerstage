package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class MatchConfig {
    public static InitVisionProcessor.AllianceColor finalAllianceColor = InitVisionProcessor.AllianceColor.BLUE;
    public static InitVisionProcessor.SideOfField finalSideOfField = InitVisionProcessor.SideOfField.BACKSTAGE;
    public static InitVisionProcessor.TeamPropLocation finalTeamPropLocation = InitVisionProcessor.TeamPropLocation.CENTER;
    public static Pose2d endOfAutonomousPose = null;
    public static double endOfAutonomousOffset;
    public static double endOfAutonomousRelativeYawDegrees;
    public static double endOfAutonomousAbsoluteYawDegrees;
    public static boolean autoHasRun=false;
}
