package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.List;

public class Shoulder {

    private double SHOULDER_VALUE_THRESHOLD = .03;
    private ShoulderStates SHOULDER_STARTING_STATE = ShoulderStates.INTAKE;

    public enum ShoulderStates {
        INTAKE (1.0),
        BACKDROP (0.0);
        private double position;
        ShoulderStates(double p) {
            this.position = p;
        }
    }

    private Servo shoulder;
    public ShoulderStates currentState = SHOULDER_STARTING_STATE;
    public double currentPosition;

    public Shoulder() {

    }

    public void init() {
        shoulder = Robot.getInstance().getHardwareMap().servo.get("shoulder");
        shoulder.setPosition(SHOULDER_STARTING_STATE.position);
    }

    public Action rotate(ShoulderStates s){
        return new Shoulder.Rotate(s);
    }

    public class Rotate implements Action {
        public final ShoulderStates targetState;
        public final double targetPosition;

        public Rotate(ShoulderStates s) {
            targetState = s;
            targetPosition = targetState.position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shoulder.setPosition(targetPosition);
            currentPosition = shoulder.getPosition();

            telemetryPacket.put("Target Shoulder State", targetState);
            telemetryPacket.put("Current Shoulder State", currentState);
            telemetryPacket.put("Target Position", targetPosition);
            telemetryPacket.put("Current Position", currentPosition);

            if (Math.abs(currentPosition - targetPosition) < SHOULDER_VALUE_THRESHOLD)
            {
                currentState = targetState;
                return false;
            } else return true;
        }
    }
}
