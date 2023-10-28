package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;

public class ActuateEndEffector extends CommandBase {

        // The subsystem the command runs on
        private final EndEffectorSubsystem endEffectorSubsystem;

        //declare target state & position
        private EndEffectorSubsystem.EndEffectorStates targetState;
        private double targetPosition;

        Telemetry telemetry;

        public ActuateEndEffector(EndEffectorSubsystem subsystem, EndEffectorSubsystem.EndEffectorStates inputState) {
            endEffectorSubsystem = subsystem;
            //save the input state, s, as the target state
            targetState = inputState;
            //get the target position from the input state
            targetPosition = targetState.position;

            //add the subsystem to the requirements
            addRequirements(endEffectorSubsystem);
        }

        @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time here in this constructor
            endEffectorSubsystem.endEffector.setPosition(targetPosition);

            //create a new telemetry packet for this command
            telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        }

        public void execute() {
            //STEP 1
            //get the current position of the servo and save it in the current position variable in the subsystem
            endEffectorSubsystem.currentPosition = endEffectorSubsystem.endEffector.getPosition();

            //STEP 2
            // add telemetry for targetState, currentState, targetPosition, and currentPosition
            // each line should look like this: telemetryPacket.put("[label]", [variable]);
            telemetry.clearAll();
            telemetry.addData("Current EndEffector State", endEffectorSubsystem.currentState);
            telemetry.addData("Current EndEffector Position", endEffectorSubsystem.currentPosition);
            telemetry.addData("Target EndEffector State: ", targetState);
            telemetry.addData("Target EndEffector Position", targetPosition);
        }

        @Override
        public boolean isFinished() {
            //STEP 3
            //check if the current position is close enough to say we are done [how would you do this?]
            // hint 1: absolute value
            // hint 2: END_EFFECTOR_POSITION_THRESHOLD is already declared at the top of this class for you to use
            boolean done = Math.abs( endEffectorSubsystem.currentPosition-targetPosition) < EndEffectorSubsystem.END_EFFECTOR_POSITION_THRESHOLD;
            //STEP 4
            //if true, then save the target state as the current state since we are now at the target and return true so the Action completes
            //if false, then return false so this action keeps getting called every loop
            if (done){
                endEffectorSubsystem.currentState = targetState;
                return true;
            } else return false;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                //what should we do if interrupted?
            }
        }
    }

