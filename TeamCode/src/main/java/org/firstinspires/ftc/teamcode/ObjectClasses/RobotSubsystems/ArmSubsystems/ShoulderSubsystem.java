package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShoulderSubsystem extends SubsystemBase {

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

    public ShoulderSubsystem(final HardwareMap hMap, final String name) {
        shoulder = hMap.servo.get("shoulder");
    }

    public void init() {
        shoulder.setPosition(SHOULDER_STARTING_STATE.position);
    }

    public void periodic(){

    }

    public Action rotate(ShoulderStates s){
        return new ShoulderSubsystem.Rotate(s);
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
