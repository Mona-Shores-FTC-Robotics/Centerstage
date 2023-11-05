package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class VisionTelemetry {

    public static void telemetryForInitProcessing() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        InitVisionProcessor.AllianceColor allianceColor =  Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor;
        InitVisionProcessor.SideOfField sideOfField =  Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().sideOfField;

        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Side of the Field", sideOfField);
        telemetry.addData("Team Prop Location", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocation());
        telemetry.addLine("");
        telemetry.addData("Left Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getLeftPercent(), 4, 1));
        telemetry.addData("Middle Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getCenterPercent(), 4, 1));
        telemetry.addData("Right Square Blue/Red Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getRightPercent(), 4, 1));

        telemetry.addData("Total Red", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentRedTotal, 4, 1));
        telemetry.addData("Total Blue", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentBlueTotal, 4, 1));
        telemetry.addData("Alliance Color Problem Flag", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColorDeterminationProblem);

        telemetry.addData("Stage Door Left Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentLeftStageDoorZone, 4, 1));
        telemetry.addData("Stage Door Right Percent", JavaUtil.formatNumber(Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().percentRightStageDoorZone, 4, 1));

        //Set the gamepads to Green if there is a problem and the driver hasn't overridden
        if ( Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColorDeterminationProblem && GamepadHandling.ManualOverrideInitSettingsFlag==false) {
            GamepadHandling.problemInInitLed();
        } else {
            //show the color of the Alliance on the drivergamepad
            if (allianceColor == InitVisionProcessor.AllianceColor.RED) {
                GamepadHandling.setRed();
            } else if (allianceColor == InitVisionProcessor.AllianceColor.BLUE) {
                GamepadHandling.setBlue();
            }
        }
    }

}
