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
    private boolean hasNotInit = true;
    private boolean isFinished = false;

    ElapsedTime shoulderTimer = new ElapsedTime();

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
            targetPosition = targetState.position;
        }

    public void init(TelemetryPacket telemetryPacket){
        hasNotInit=false;
        shoulderTimer.reset();
        Robot.getInstance().getShoulderSubsystem().setTargetState(targetState);
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetState.position);
        telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetryPacket.put("Target Shoulder State", targetState);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) init(telemetryPacket);

        telemetryPacket.put("Shoulder Timer", shoulderTimer.milliseconds());

        if (isFinished(telemetryPacket)) {
            //returns fall because the Action should no longer run
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
            return false;
        }
        // else the action should continue to run
        else {
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
            return true;
        }
    }

    public boolean isFinished(TelemetryPacket telemetryPacket) {
        boolean done = shoulderTimer.milliseconds() > ShoulderSubsystem.shoulderParameters.SHOULDER_ROTATE_THRESHOLD_MILLISECONDS;
        if (done)
        {
            telemetryPacket.put("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
            Robot.getInstance().getShoulderSubsystem().setCurrentState(targetState);
            return true;
        } else {
            return false;
        }
    }
}
