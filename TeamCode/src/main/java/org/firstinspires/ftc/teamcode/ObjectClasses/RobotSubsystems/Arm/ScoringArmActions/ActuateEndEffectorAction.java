package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;

public class ActuateEndEffectorAction implements Action {

    //declare target state & position
    private GripperSubsystem.GripperStates targetState;
    private double targetPosition;

    public ActuateEndEffectorAction(GripperSubsystem.GripperStates inputState) {
        //save the input state, s, as the target state
        targetState = inputState;
        //get the target position from the input state
        targetPosition = targetState.position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.getInstance().getGripperSubsystem().endEffector.setPosition(targetPosition);
        telemetryPacket.put("Current EndEffector State", Robot.getInstance().getGripperSubsystem().currentState);
        telemetryPacket.put("Target EndEffector State: ", targetState);
        Robot.getInstance().getGripperSubsystem().currentState = targetState;
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        return false;
    }
}

