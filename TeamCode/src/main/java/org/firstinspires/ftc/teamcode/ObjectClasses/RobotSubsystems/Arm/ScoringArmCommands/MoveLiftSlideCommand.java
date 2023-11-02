package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

public class MoveLiftSlideCommand extends CommandBase {
    //Declare and set a private final timeout threshold for the command called TIMEOUT_TIME_SECONDS - I suggest 3 seconds for now

    //Declare a final local variable to hold the command subsystem

    //Declare private variables for currentTicks, targetTicks, and targetState

    //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish

    //declare a timeout boolean variable


    public MoveLiftSlideCommand(LiftSlideSubsystem subsystem, LiftSlideSubsystem.LiftStates inputState) {
        //save the subsystem as your local final variable declared above

        //save the input state as the local targetstate variable

        //make a new ElapsedTime() object and save it as your timeoutTimer

        //add the subsystem to the requirements
    }

        @Override
        public void initialize() {
            //initialize is called once when the command is first run

            //Set the targetState of the subsystem to the local targetState
            //liftSlideSubsystem.[use the dot to look for the method you need];

            //Set the targetTicks of the subsystem based on the targetState - this one is a freebie
            //liftSlideSubsystem.setTargetTicks(liftSlideSubsystem.getTargetState().ticks);

            //reset the timeoutTimer

            //set the timeout to false since we have not timed out yet

            //get currentTicks from the subsystem and save locally as currentTicks

            //get targetTicks from the subsystem and save locally as targetTicks

            //Check if targetTicks is greater than liftSlideSubsystem.MAX_TARGET_TICKS and if it is set the target to the max
            //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot


            //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
            //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot

            //if the target ticks are higher than the current ticks, then set the liftSlide power to EXTENSION_LIFT_POWER

            //if the target ticks are lower than the current ticks, then set the liftSlide power to RETRACTION_POWER

            //Set the target position of the liftSlide of the liftSlideSubsystem using the targetTicks

            //set the liftSlide motor to RUN TO POSITION
    }

    public void execute() {
        //The execute of this command is empty because we don't need to do anything while the command is active
        //the RUN_TO_POSITION we set in the initialize means that the lift motor is going to automatically try to reach the target
        //there is telemetry in the periodic() method of the lift subsystem that will give us updates in the dashboard about the current ticks, current state, target state, and target ticks
    }

    @Override
    public boolean isFinished() {
        // Declare a boolean variable called finished

        // Compare the currentTicks to the targetTicks to a threshold (LIFT_HEIGHT_TICK_THRESHOLD) and save as the boolean
        // For example, say our target is 2000 ticks and we are at 1997 - we would want that to count as being close enough to finished

        //if the command is finished, then return true (meaning the command will stop running once it runs end() one time)

        //compare the elapsed time to a timeout threshold and if the elapsed time is greater than the threshold return true

        //if the command isn't finished and the command isn't timed out then return false because the command should still run
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        TelemetryPacket p = new TelemetryPacket();
        //If !timeout and !interrupted, then report the command finished and change the currentState
        {
            //Report the command finished - this is pre-written for you in the comment below
            //p.addLine("LiftSlide Move COMPLETE From " + liftSlideSubsystem.getCurrentState() + " to " + liftSlideSubsystem.getTargetState() + " in " + String.format("%.2f", timeoutTimer.seconds()) + " seconds");

            //set the currentState in the subsystem to the target state - you have to write this
        }

        //if timeout is true, then tell the user
        {
            //Put the target state in the packet - the next two lines can just be uncommented
            //p.addLine("LiftSlide Move TIMEOUT");
            //p.put("Timeout Timer", timeoutTimer.seconds());
        }
        //if interrupted, then tell the user
        if (interrupted){
            //Put the interruption telemetry in the packet - just uncomment the next line
            //p.addLine("LiftSlide Move INTERRUPTED From " + liftSlideSubsystem.getCurrentState() + " to " + liftSlideSubsystem.getTargetState() + " at " + timeoutTimer.seconds() + " seconds");
        }
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
