package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.List;

//todo Write the code for an action to rotate the shoulder
@Config
public class Shoulder {

    public static class ShoulderParameters {
        private double SHOULDER_POSITION_THRESHOLD = .03;
        private ShoulderStates SHOULDER_STARTING_STATE = ShoulderStates.INTAKE;
        public double INTAKE_POSITION = 1.0;
        public double BACKDROP_POSITION = 0.0;
    }

    public enum ShoulderStates {
        INTAKE (1.0),
        BACKDROP (0.0);
        private double position;
        ShoulderStates(double pos) {
            this.position = pos;
        }
    }

    private Servo shoulder;

    //declare the currentState variable and set it to the starting state
    //declare the current position variable

    public Shoulder() {

    }

    //code to run when the robot is turned on
    public void init() {
        shoulder = Robot.getInstance().getHardwareMap().servo.get("shoulder");
        //set the initial position of the servo to the current position
        //it should be like this: shoulder.setPosition([current position variable]);
    }

    public Action rotate(ShoulderStates s){
         return new Shoulder.Rotate(s);
    }

    public class Rotate implements Action {
        //declare target state
        //declare target position

        public Rotate(ShoulderStates s) {
            //save the input state as the target state
            //get the target position from the enum
            //set the Servo position to the target - we only have to set the servo position one time
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //STEP 1
            //get the current position of the shoulder servo and save it in the current position variable

            //STEP 2
            // add telemetry for targetState, currentState, targetPosition, and currentPosition
            // telemetryPacket.put("[label]", [variable]);

            //STEP 3
            //check if the current position is close enough to say we are done
            //if true, then save the targetstate as the current state since we are now at the target and return true so the Action completes
            //if false, then return false so this action keeps getting called

            return false;
        }
    }
}
