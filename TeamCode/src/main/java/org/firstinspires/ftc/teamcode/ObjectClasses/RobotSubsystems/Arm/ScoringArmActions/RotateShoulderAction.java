package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderAction implements Action {

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;
    private boolean hasNotInit = true;
    private boolean isFinished = false;

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
            targetPosition = targetState.position;
        }

    public void init(){
        hasNotInit=false;
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetState.position);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) init();

        Robot.getInstance().getShoulderSubsystem().currentPosition =
                Robot.getInstance().getShoulderSubsystem().shoulder.getPosition();

        telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetryPacket.put("Current Position", Robot.getInstance().getShoulderSubsystem().currentPosition);
        telemetryPacket.put("Target Shoulder State", targetState);
        telemetryPacket.put("Target Position", targetPosition);

        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        if (isFinished()) {
            //returns fall because the Action should no longer run
            return false;
        }
        // else the action should continue to run
        else return true;
    }

    public boolean isFinished() {
        boolean done = Math.abs(Robot.getInstance().getShoulderSubsystem().currentPosition-targetPosition) < ShoulderSubsystem.shoulderParameters.SHOULDER_VALUE_THRESHOLD;
        if (done)
        {
            Robot.getInstance().getShoulderSubsystem().currentState = targetState;
            return true;
        } else return false;
    }
}
