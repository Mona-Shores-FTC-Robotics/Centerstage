package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class Shoulder {

    private double SHOULDER_VALUE_THRESHOLD = .03;

    public enum ShoulderStates {
        INTAKE (1.0),
        BACKDROP (0);
        private double value;
        ShoulderStates(double value) {
            this.value = value;
        }

    }

    private Servo shoulder;
    private LinearOpMode activeOpMode;
    private ShoulderStates targetState = ShoulderStates.INTAKE;
    private ShoulderStates currentState = ShoulderStates.INTAKE;
    private double currentPosition;
    private double targetPosition;

    public Shoulder() {

    }

    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        shoulder = Robot.getInstance().getHardwareMap().servo.get("shoulder");
        targetState = currentState = ShoulderStates.INTAKE;
        shoulder.setPosition(targetState.value);
    }

    public Action rotateToIntake(){
        return new Shoulder.RotateToIntake();
    }

    public Action rotateToBackdrop(){
        return new Shoulder.RotateToBackdrop();
    }

    public class RotateToIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentPosition = shoulder.getPosition();
            targetState = ShoulderStates.INTAKE;
            targetPosition = targetState.value;

            shoulder.setPosition(targetPosition);

            telemetryPacket.put("Target Shoulder State", targetState);
            telemetryPacket.put("Current Shoulder State", currentState);
            telemetryPacket.put("Target Position", targetPosition);
            telemetryPacket.put("Current Position", currentPosition);
            if (Math.abs(currentPosition - targetPosition) < SHOULDER_VALUE_THRESHOLD)
            {
                currentState = targetState;
                return true;
            } else return false;
        }
    }

    public class RotateToBackdrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentPosition = shoulder.getPosition();
            targetState = ShoulderStates.BACKDROP;
            targetPosition = targetState.value;

            shoulder.setPosition(targetPosition);

            telemetryPacket.put("Target Shoulder Position", targetState);
            telemetryPacket.put("Target Value", targetPosition);
            telemetryPacket.put("Current Value", currentPosition);
            if (Math.abs(currentPosition - targetPosition) < SHOULDER_VALUE_THRESHOLD)
            {
                currentState = targetState;
                return true;
            } else return false;
        }
    }
}
