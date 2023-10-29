package org.firstinspires.ftc.teamcode.ObjectClasses.Telemetry;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.IntakeCommands.ChangeIntakeState;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.IntakeCommands.ChangeIntakeStateVelocity;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IntakeSubsystem;

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
        telemetry.addLine("A Button - Shoulder Servo");
        telemetry.addLine("Y Button - Claw Servo");
        telemetry.addLine("X Button - Lift Motor");
    }

}
