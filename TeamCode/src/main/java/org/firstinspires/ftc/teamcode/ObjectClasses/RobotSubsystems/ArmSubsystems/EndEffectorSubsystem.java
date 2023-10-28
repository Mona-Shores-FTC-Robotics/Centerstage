package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public final class EndEffectorSubsystem extends SubsystemBase {

    public static EndEffectorStates END_EFFECTOR_STARTING_STATE = EndEffectorStates.CLOSED;
    public static double END_EFFECTOR_POSITION_THRESHOLD = .02;
    public static double OPEN_POSITION = 1;
    public static double CLOSED_POSITION = 0;


    public enum EndEffectorStates {
        CLOSED (0),
        OPEN (1);
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
        EndEffectorStates.CLOSED.SetState(CLOSED_POSITION);
        EndEffectorStates.OPEN.SetState(OPEN_POSITION);
    }

}
