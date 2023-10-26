package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class EndEffector {

    private double END_EFFECTOR_POSITION_THRESHOLD = .03;

    public enum EndEffectorStates {
        CLOSED (1),
        OPEN (0);

        private double endEffectorPosition;

        private EndEffectorStates(double endEffectorPosition) {
            this.endEffectorPosition = endEffectorPosition;
        }
    }

    public EndEffectorStates currentEndEffectorState;

    private Servo endEffector;
    private LinearOpMode activeOpMode;
    private EndEffector.EndEffectorStates targetState = EndEffectorStates.OPEN;
    private EndEffector.EndEffectorStates currentState = EndEffectorStates.OPEN;
    private double currentPosition;
    private double targetPosition;

    public EndEffector() {

    }

    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        HardwareMap ahwMap = Robot.getInstance().getHardwareMap();
        endEffector = ahwMap.servo.get("endeffector");
        currentEndEffectorState = EndEffectorStates.OPEN;
        endEffector.setPosition(currentEndEffectorState.endEffectorPosition);
    }

    public Action openEndEffector(){
        return new EndEffector.OpenEndEffector();
    }

    public Action closeEndEffector(){
        return new EndEffector.CloseEndEffector();
    }

    public class OpenEndEffector implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentPosition = endEffector.getPosition();
            targetState = EndEffectorStates.OPEN;
            targetPosition = targetState.endEffectorPosition;

            endEffector.setPosition(targetPosition);

            telemetryPacket.put("Target EndEffector State", targetPosition);
            telemetryPacket.put("Target Position", targetPosition);
            telemetryPacket.put("Current Position", currentPosition);
            if (Math.abs(currentPosition - targetPosition) < END_EFFECTOR_POSITION_THRESHOLD)
            {
                return true;
            } else return false;
        }
    }

    public class CloseEndEffector implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentPosition = endEffector.getPosition();
            targetState = EndEffectorStates.CLOSED;
            targetPosition = targetState.endEffectorPosition;

            endEffector.setPosition(targetPosition);

            telemetryPacket.put("Target EndEffector State", targetPosition);
            telemetryPacket.put("Target Position", targetPosition);
            telemetryPacket.put("Current Position", currentPosition);
            if (Math.abs(currentPosition - targetPosition) < END_EFFECTOR_POSITION_THRESHOLD)
            {
                return true;
            } else return false;
        }
    }
}
