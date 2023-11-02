package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
//write command to move the lift to the height that corresponds with the inputState
public class MoveLiftSlideCommand extends CommandBase {
    //Declare and set a private final timeout threshold for the command called TIMEOUT_TIME_SECONDS - I suggest 3 seconds for now
    private final double TIMEOUT_TIME_SECONDS = 3;

    //Declare a final local variable to hold the command subsystem
    private final LiftSlideSubsystem liftSlideSubsystem;
    //Declare private variables for currentTicks, targetTicks, and targetState
    private int targetTicks, currentTicks;
    private LiftSlideSubsystem.LiftStates targetState;
    //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish
    private ElapsedTime timeoutTimer;
    //declare a timeout boolean variable
    private boolean timeout;

    public MoveLiftSlideCommand(LiftSlideSubsystem subsystem, LiftSlideSubsystem.LiftStates inputState) {
        //save the subsystem as your local final variable declared above
        liftSlideSubsystem  = subsystem;
        //save the input state as the local targetstate variable
        targetState = inputState;
        //make a new ElapsedTime() object and save it as your timeoutTimer
        timeoutTimer = new ElapsedTime();
        //add the subsystem to the requirements
        addRequirements(liftSlideSubsystem);
    }

        @Override
        public void initialize() {
            //initialize is called once when the command is first run

            //Set the targetState of the subsystem to the local targetState
            liftSlideSubsystem.setTargetState(targetState);

            //Set the targetTicks of the subsystem based on the targetState - this one is a freebie
            liftSlideSubsystem.setTargetTicks(liftSlideSubsystem.getTargetState().ticks);

            //reset the timeoutTimer
            timeoutTimer.reset();
            //set the timeout to false since we have not timed out yet
            timeout = false;
            //get currentTicks from the subsystem and save locally as currentTicks
            currentTicks = liftSlideSubsystem.getCurrentTicks();
            //get targetTicks from the subsystem and save locally as targetTicks
            targetTicks = liftSlideSubsystem.getTargetTicks();
            //Check if targetTicks is greater than liftSlideSubsystem.MAX_TARGET_TICKS and if it is set the target to the max
            //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot
            boolean overMax = targetTicks >  liftSlideSubsystem.MAX_TARGET_TICKS;
            if (overMax) {
                targetTicks =  liftSlideSubsystem.MAX_TARGET_TICKS;
            }
            //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
            //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot
            boolean underMin = targetTicks < liftSlideSubsystem.MIN_TARGET_TICKS;
            if (underMin) {
                targetTicks = liftSlideSubsystem.MIN_TARGET_TICKS;
            }
            //if the target ticks are higher than the current ticks, then set the liftSlide power to EXTENSION_LIFT_POWER
            if (targetTicks > currentTicks) {
                liftSlideSubsystem.liftSlide.setPower(EXTENSION_LIFT_POWER);
            }
            //if the target ticks are lower than the current ticks, then set the liftSlide power to RETRACTION_POWER
            if (targetTicks < currentTicks) {
                liftSlideSubsystem.liftSlide.setPower(RETRACTION_LIFT_POWER);
            }
            //Set the target position of the liftSlide of the liftSlideSubsystem using the targetTicks
            liftSlideSubsystem.liftSlide.setTargetPosition(targetTicks);
            //set the liftSlide motor to RUN TO POSITION
            liftSlideSubsystem.liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void execute() {
        //The execute of this command is empty because we don't need to do anything while the command is active
        //the RUN_TO_POSITION we set in the initialize means that the lift motor is going to automatically try to reach the target
        //there is telemetry in the periodic() method of the lift subsystem that will give us updates in the dashboard about the current ticks, current state, target state, and target ticks
    }

    @Override
    public boolean isFinished() {
        // Declare a boolean variable called finished
        boolean finished;
        // Compare the currentTicks to the targetTicks to a threshold (LIFT_HEIGHT_TICK_THRESHOLD) and save as the boolean
        // For example, say our target is 2000 ticks and we are at 1997 - we would want that to count as being close enough to finished
        if ( Math.abs (currentTicks - targetTicks) > (LIFT_HEIGHT_TICK_THRESHOLD) ) {

            //if the command is finished, then return true (meaning the command will stop running once it runs end() one time)
            return true ;
        }
        //compare the elapsed time to a timeout threshold and if the elapsed time is greater than the threshold return true
        if (timeoutTimer.seconds() > TIMEOUT_TIME_SECONDS) {
            timeout = true;
            return true ;
        }
        //if the command isn't finished and the command isn't timed out then return false because the command should still run
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        TelemetryPacket p = new TelemetryPacket();
        //If !timeout and !interrupted, then report the command finished and change the currentState
        if (!timeout && !interrupted)
        {
            //Report the command finished - this is pre-written for you in the comment below
            p.addLine("LiftSlide Move COMPLETE From " + liftSlideSubsystem.getCurrentState() + " to " + liftSlideSubsystem.getTargetState() + " in " + String.format("%.2f", timeoutTimer.seconds()) + " seconds");

            //set the currentState in the subsystem to the target state - you have to write this
            liftSlideSubsystem.setCurrentState(targetState);
        }

        //if timeout is true, then tell the user
        if (timeout )
        {
            //Put the target state in the packet - the next two lines can just be uncommented
            p.addLine("LiftSlide Move TIMEOUT");
            p.put("Timeout Timer", timeoutTimer.seconds());
        }
        //if interrupted, then tell the user
        if (interrupted){
            //Put the interruption telemetry in the packet - just uncomment the next line
            p.addLine("LiftSlide Move INTERRUPTED From " + liftSlideSubsystem.getCurrentState() + " to " + liftSlideSubsystem.getTargetState() + " at " + timeoutTimer.seconds() + " seconds");
        }
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
