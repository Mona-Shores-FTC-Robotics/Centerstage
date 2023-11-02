package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShoulderSubsystem extends SubsystemBase {

    public static class ShoulderParameters {
        public ShoulderStates SHOULDER_STARTING_STATE = ShoulderStates.INTAKE;
        public double SHOULDER_VALUE_THRESHOLD = .03;
        public double INTAKE_VALUE = 0;
        public double BACKDROP_VALUE = 1;
    }

    public static ShoulderParameters shoulderParameters = new ShoulderParameters();

    public enum ShoulderStates {
        INTAKE (0),
        BACKDROP (1);
        public double position;
        ShoulderStates(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public Servo shoulder;
    public ShoulderStates currentState;
    public double currentPosition;

    public ShoulderSubsystem(final HardwareMap hMap, final String name) {
        shoulder = hMap.servo.get("shoulder");
    }

    public void init() {
        ShoulderStates.BACKDROP.SetState(shoulderParameters.BACKDROP_VALUE);
        ShoulderStates.INTAKE.SetState(shoulderParameters.INTAKE_VALUE);

        currentState = shoulderParameters.SHOULDER_STARTING_STATE;
        currentPosition = currentState.position;
//        shoulder.setPosition(currentPosition);
    }

    public void periodic(){
        ShoulderStates.INTAKE.SetState(shoulderParameters.INTAKE_VALUE);
        ShoulderStates.BACKDROP.SetState(shoulderParameters.BACKDROP_VALUE);

    }

}
