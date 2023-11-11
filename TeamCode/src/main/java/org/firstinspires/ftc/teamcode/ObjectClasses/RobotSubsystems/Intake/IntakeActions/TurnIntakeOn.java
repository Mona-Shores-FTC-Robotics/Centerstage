package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem.intakeParameters;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class TurnIntakeOn implements Action {

    public TurnIntakeOn() {
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getIntakeSubsystem().intake.setDirection(DcMotor.Direction.FORWARD);
        Robot.getInstance().getIntakeSubsystem().intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().getIntakeSubsystem().currentState = IntakeSubsystem.IntakeStates.INTAKE_ON;
        Robot.getInstance().getIntakeSubsystem().intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.getInstance().getIntakeSubsystem().intake.setPower(intakeParameters.INTAKE_ON_POWER);
        return false;
    }
}