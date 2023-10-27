package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.MecanumDriveMona;

//todo Write the code for an action to actuate the end effector
@Config
public final class EndEffector {

    public static class EndEffectorParameters {
        public double END_EFFECTOR_POSITION_THRESHOLD = .03;
        public EndEffectorStates END_EFFECTOR_STARTING_STATE = EndEffectorStates.CLOSED;
        public double CLOSED_POSITION = 1.0;
        public double OPEN_POSITION = 0.0;
    }

    public static EndEffectorParameters endEffectorParameters = new EndEffectorParameters();

    public enum EndEffectorStates {
        CLOSED (endEffectorParameters.CLOSED_POSITION),
        OPEN (endEffectorParameters.OPEN_POSITION);

        private double position;

        private EndEffectorStates(double pos) {
            this.position = pos;
        }
    }

    private Servo endEffector;

    //declare the currentState variable and set it to the starting state
    EndEffectorStates currentState = endEffectorParameters.END_EFFECTOR_STARTING_STATE;
    //declare the current position variable and set it to the starting position
    double currentPosition = currentState.position;
    public EndEffector() {

    }

    public void init() {
        endEffector = Robot.getInstance().getHardwareMap().servo.get("endeffector");
        //set the initial position of the servo to the current position
        //endEffector.setPosition([current position variable]);
        endEffector.setPosition(currentPosition);

    }

    public Action actuate(EndEffectorStates s){
        return new EndEffector.Actuate(s);
    }

    public class Actuate implements Action {
        //declare target state
        EndEffectorStates targetState;
        //declare target position
        double targetPosition;

        public Actuate(EndEffectorStates inputState) {
            //save the input state, s, as the target state
            targetState = inputState;
            //get the target position from the input state
            targetPosition = targetState.position;
            //set the Servo position to the target - we only have to set the servo position one time here in this constructor
            endEffector.setPosition (targetPosition);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //STEP 1
            //get the current position of the servo and save it in the current position variable
            currentPosition = endEffector.getPosition();
            //STEP 2
            // add telemetry for targetState, currentState, targetPosition, and currentPosition
            // each line should look like this: telemetryPacket.put("[label]", [variable]);
            telemetryPacket.put("Target State: ", targetState);
            telemetryPacket.put("Current State", currentState);
            telemetryPacket.put("targetPosition", targetPosition);
            telemetryPacket.put("currentPosition", currentPosition);

            //STEP 3
            //check if the current position is close enough to say we are done [how would you do this?]
            // hint 1: absolute value
            // hint 2: END_EFFECTOR_POSITION_THRESHOLD is already declared at the top of this class for you to use
            boolean done = Math.abs(currentPosition-targetPosition) < endEffectorParameters.END_EFFECTOR_POSITION_THRESHOLD;

            //STEP 4
            //if true, then save the target state as the current state since we are now at the target and return true so the Action completes
            //if false, then return false so this action keeps getting called every loop
            if (done) return true; else return false;
        }
    }
}
