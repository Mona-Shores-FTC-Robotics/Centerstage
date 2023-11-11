package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;

public class ActuateEndEffectorAction implements Action {

    //declare target state & position
    private EndEffectorSubsystem.EndEffectorStates targetState;
    private double targetPosition;
    private boolean hasNotInit = true;
    private boolean finished = false;

    Telemetry telemetry;

    public ActuateEndEffectorAction(EndEffectorSubsystem.EndEffectorStates inputState) {
        //save the input state, s, as the target state
        targetState = inputState;
        //get the target position from the input state
        targetPosition = targetState.position;
    }

    public void init() {
        hasNotInit=false;
        //set the Servo position to the target - we only have to set the servo position one time here in this constructor
        Robot.getInstance().getEndEffectorSubsystem().endEffector.setPosition(targetPosition);

        //create a new telemetry packet for this command
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) init();

        //STEP 1
        //get the current position of the servo and save it in the current position variable in the subsystem
        Robot.getInstance().getEndEffectorSubsystem().currentPosition =
                Robot.getInstance().getEndEffectorSubsystem().endEffector.getPosition();

        //STEP 2
        // add telemetry for targetState, currentState, targetPosition, and currentPosition
        // each line should look like this: telemetryPacket.put("[label]", [variable]);
        telemetry.clearAll();
        telemetry.addData("Current EndEffector State", Robot.getInstance().getEndEffectorSubsystem().currentState);
        telemetry.addData("Current EndEffector Position", Robot.getInstance().getEndEffectorSubsystem().currentPosition);
        telemetry.addData("Target EndEffector State: ", targetState);
        telemetry.addData("Target EndEffector Position", targetPosition);

        if (isFinished()) {
            return false;
        } else return true;
    }

    public boolean isFinished() {
        //STEP 3
        //check if the current position is close enough to say we are done [how would you do this?]
        // hint 1: absolute value
        // hint 2: END_EFFECTOR_POSITION_THRESHOLD is already declared at the top of this class for you to use
        finished = Math.abs( Robot.getInstance().getEndEffectorSubsystem().currentPosition-targetPosition) < EndEffectorSubsystem.END_EFFECTOR_POSITION_THRESHOLD;
        //STEP 4
        //if true, then save the target state as the current state since we are now at the target and return true so the Action completes
        //if false, then return false so this action keeps getting called every loop
        if (finished){
            Robot.getInstance().getEndEffectorSubsystem().currentState = targetState;
            return true;
        } else return false;
    }
}

