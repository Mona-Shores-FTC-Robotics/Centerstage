package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderAction implements Action {
    private ShoulderSubsystem.ShoulderStates targetState;

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
        }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetryPacket.put("Target Shoulder State", targetState);
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetState.position);
        Robot.getInstance().getShoulderSubsystem().setCurrentState(targetState);
        return false;
    }
}
