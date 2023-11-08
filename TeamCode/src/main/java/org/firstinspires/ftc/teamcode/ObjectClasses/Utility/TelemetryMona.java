package org.firstinspires.ftc.teamcode.ObjectClasses.Utility;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

public class TelemetryMona {

    static Telemetry telemetry;

    public static void intakeTestingButtons() {
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");
        telemetry.addLine("Right Bumper activate velocity control - rightY controls velocity");
        telemetry.addLine("Y Button - Intake On");
        telemetry.addLine("X Button - Intake Reverse");
    }

    public static void scoringArmTestingButtons() {
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");
//        telemetry.addLine("A Button - Shoulder Servo");
//        telemetry.addLine("Y Button - Claw Servo");
        telemetry.addLine("X Button - Lift Motor LOW");
        telemetry.addLine("Y Button - Lift Motor MID");
        telemetry.addLine("B Button - Lift Motor HIGH");
    }

}
