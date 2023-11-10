package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem.intakeParameters;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class TurnIntakeOff implements Action {

    public TurnIntakeOff() {
        //TODO finish this...
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getIntakeSubsystem().currentState = IntakeSubsystem.IntakeStates.INTAKE_OFF;
        Robot.getInstance().getIntakeSubsystem().intake.setPower(intakeParameters.INTAKE_OFF_POWER);
        Robot.getInstance().getIntakeSubsystem().intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.getInstance().getIntakeSubsystem().intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return false;
    }
}