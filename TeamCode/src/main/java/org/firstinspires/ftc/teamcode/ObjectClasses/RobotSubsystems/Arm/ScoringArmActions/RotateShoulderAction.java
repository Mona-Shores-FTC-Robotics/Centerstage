package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderAction implements Action {

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
            targetPosition = targetState.position;
        }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getShoulderSubsystem().setTargetState(targetState);
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetPosition);
        telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetryPacket.put("Target Shoulder State", targetState);
        Robot.getInstance().getShoulderSubsystem().setCurrentState(targetState);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        return false;
    }
}
