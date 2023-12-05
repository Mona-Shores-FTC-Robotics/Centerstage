package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;

public class ActuatePixelPusherAction implements Action {

    //declare target state & position
    private PixelPusherSubsystem.PixelPusherStates targetState;
    private double targetPosition;

    public ActuatePixelPusherAction(PixelPusherSubsystem.PixelPusherStates inputState) {
        //save the input state, s, as the target state
        targetState = inputState;
        //get the target position from the input state
        targetPosition = targetState.position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Current PixelPusher State", Robot.getInstance().getGripperSubsystem().currentState);
        telemetryPacket.put("Target PixelPusher State: ", targetState);
        Robot.getInstance().getPixelPusherSubsystem().pixelPusher.setPosition(targetPosition);
        Robot.getInstance().getPixelPusherSubsystem().currentState = targetState;
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        return false;
    }
}

