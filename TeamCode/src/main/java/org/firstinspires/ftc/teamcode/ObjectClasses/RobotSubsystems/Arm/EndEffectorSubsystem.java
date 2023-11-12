package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class EndEffectorSubsystem extends SubsystemBase {

    public static EndEffectorStates END_EFFECTOR_STARTING_STATE = EndEffectorStates.OPEN;
    public static double END_EFFECTOR_POSITION_THRESHOLD = .02;
    public static double OPEN_POSITION = .4;
    public static double CLOSED_POSITION = .55;


    public enum EndEffectorStates {
        CLOSED (.55),
        OPEN (.4);
        public double position;
        EndEffectorStates(double pos) {
            this.position = pos;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public Servo endEffector;
    public EndEffectorStates currentState;
    public double currentPosition;

    public EndEffectorSubsystem(final HardwareMap hMap, final String name) {
        endEffector = hMap.get(Servo.class, name);
    }

    public void init() {
        EndEffectorStates.CLOSED.SetState(CLOSED_POSITION);
        EndEffectorStates.OPEN.SetState(OPEN_POSITION);
        currentState= END_EFFECTOR_STARTING_STATE;
        currentPosition = currentState.position;
        endEffector.setPosition(currentPosition);

    }

    public void periodic(){

    }

}
