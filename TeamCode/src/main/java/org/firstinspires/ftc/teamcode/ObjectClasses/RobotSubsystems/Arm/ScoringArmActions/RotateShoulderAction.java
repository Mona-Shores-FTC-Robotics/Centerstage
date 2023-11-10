package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderAction implements Action {

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;
    private boolean hasNotInit = true;
    private boolean isFinished = false;

    Telemetry telemetry;

    public RotateShoulderAction(ShoulderSubsystem.ShoulderStates inputState) {
            targetState = inputState;
            targetPosition = targetState.position;
        }

    public void init(){
        hasNotInit=false;
        Robot.getInstance().getShoulderSubsystem().shoulder.setPosition(targetState.position);
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) init();

        Robot.getInstance().getShoulderSubsystem().currentPosition =
                Robot.getInstance().getShoulderSubsystem().shoulder.getPosition();

        telemetry.addData("Current Shoulder State", Robot.getInstance().getShoulderSubsystem().currentState);
        telemetry.addData("Current Position", Robot.getInstance().getShoulderSubsystem().currentPosition);
        telemetry.addData("Target Shoulder State", targetState);
        telemetry.addData("Target Position", targetPosition);

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
