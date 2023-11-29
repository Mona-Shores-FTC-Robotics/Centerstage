package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class MatchConfig {
    public static InitVisionProcessor.AllianceColor finalAllianceColor = InitVisionProcessor.AllianceColor.BLUE;
    public static InitVisionProcessor.SideOfField finalSideOfField = InitVisionProcessor.SideOfField.BACKSTAGE;
    public static InitVisionProcessor.TeamPropLocation finalTeamPropLocation = InitVisionProcessor.TeamPropLocation.LEFT;
    public static Pose2d endOfAutonomousPose = null;
    public static double endOfAutonomousOffset;
    public static double endOfAutonomousRelativeYawDegrees;
    public static double endOfAutonomousAbsoluteYawDegrees;
    public static boolean autoHasRun=false;

    public static ElapsedTime OpModeTimer;
    public static ElapsedTime loopTimer;
    public static ElapsedTime timestampTimer;
    public static boolean robot19429;

    public static TelemetryPacket telemetryPacket;

    public static void LoopDriverStationTelemetry() {
        //Print our color,
        Robot.getInstance().getActiveOpMode().telemetry.addData("Alliance Color", MatchConfig.finalAllianceColor);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("OpMode Time " + JavaUtil.formatNumber(MatchConfig.OpModeTimer.seconds(), 4, 1) + " / 120 seconds");
        Robot.getInstance().getActiveOpMode().telemetry.addData("Loop Time ", JavaUtil.formatNumber(MatchConfig.loopTimer.milliseconds(), 4, 1));

        Robot.getInstance().getActiveOpMode().telemetry.addLine();
        Robot.getInstance().getActiveOpMode().telemetry.addData("Current Pose", "X %5.2f, Y %5.2f, heading %5.2f ",
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log());

        Robot.getInstance().getActiveOpMode().telemetry.addLine();
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Gyro Yaw Angle Absolute (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees, 5, 2));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Gyro Yaw Angle Relative (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees, 5, 2));
        Robot.getInstance().getActiveOpMode().telemetry.addData("Current Drive/Turn/Strafe", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                Robot.getInstance().getDriveSubsystem().mecanumDrive.current_drive_ramp,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.current_strafe_ramp,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.current_turn_ramp);

        Robot.getInstance().getActiveOpMode().telemetry.update();
    }

}
