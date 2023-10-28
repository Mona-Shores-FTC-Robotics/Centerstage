package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShoulderSubsystem extends SubsystemBase {

    public static class ShoulderParameters {
        public ShoulderStates SHOULDER_STARTING_STATE = ShoulderStates.INTAKE;
        public double SHOULDER_VALUE_THRESHOLD = .03;
    }

    public static ShoulderParameters shoulderParameters;

    public enum ShoulderStates {
        INTAKE (0),
        BACKDROP (1);
        public double position;
        ShoulderStates(double p) {
            this.position = p;
        }
    }

    public Servo shoulder;
    public ShoulderStates currentState;
    public double currentPosition;

    public ShoulderSubsystem(final HardwareMap hMap, final String name) {
        shoulder = hMap.servo.get("shoulder");
    }

    public void init() {
        shoulderParameters = new ShoulderParameters();
        currentState = shoulderParameters.SHOULDER_STARTING_STATE;
        currentPosition = currentState.position;
        shoulder.setPosition(currentPosition);
    }

    public void periodic(){

    }

}
