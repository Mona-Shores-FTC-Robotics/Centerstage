package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;

public class MoveLiftSlide extends CommandBase {

    private double TIMEOUT_TIME_SECONDS = 1.5;

    //Declare the local variable to hold the liftsubsystem
    private final LiftSlideSubsystem liftSlideSubsystem;

    //declare the targetState, targetTicks, currentTicks
    private LiftSlideSubsystem.LiftStates targetState;
    private int targetTicks;
    private int currentTicks;

    //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish
    private ElapsedTime timeoutTimer;

    //declare a timeout boolean
    boolean timeout;

    public MoveLiftSlide(LiftSlideSubsystem subsystem, LiftSlideSubsystem.LiftStates inputState) {
        liftSlideSubsystem = subsystem;
        targetState = inputState;
        targetTicks = targetState.ticks;
        timeoutTimer = new ElapsedTime();
        //add the subsystem to the requirements
        addRequirements(liftSlideSubsystem);
    }

    @Override
    public void initialize() {
        //reset the timer
        timeoutTimer.reset();
        //set the timeout to false since we have not timed out yet
        timeout=false;

        //get the currentTicks from the subsystem
        currentTicks = liftSlideSubsystem.getCurrentTicks();

        //Check if targetTicks is greater than MAX_TARGET_TICKS and if it is set the target to the max
        //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot
        if (targetTicks> liftSlideSubsystem.MAX_TARGET_TICKS)
        {
            targetTicks=liftSlideSubsystem.MAX_TARGET_TICKS;
        }

        //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
        //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot
        if (targetTicks < liftSlideSubsystem.MIN_TARGET_TICKS)
        {
            targetTicks=liftSlideSubsystem.MIN_TARGET_TICKS;
        }

        //if the target ticks are higher than the current ticks, then use EXTENSION_POWER
        if (targetTicks > currentTicks) {
            liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
        }

        //if the target ticks are lower than the current ticks, then use RETRACTION_POWER
        if (targetTicks < currentTicks) {
            liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.RETRACTION_LIFT_POWER);
        }

        //Set the target position using the targetTicks
        liftSlideSubsystem.liftSlide.setTargetPosition(targetTicks);

        //set the lift motor to RUN TO POSITION - this might not be necessary
        liftSlideSubsystem.liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Make a new TelemetryPacket
        TelemetryPacket p = new TelemetryPacket();

        //Put the target state in the packet
        p.put("Target LiftSlide State", targetState);
        //put the target ticks in the packet
        p.put("Target Ticks", targetTicks);

        //send the packet to the dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    public void execute() {
        //The execute of this command is empty because we don't need to do anything while the command is active
        //the RUN_TO_POSITION we set in the intialize means that the lift motor is going to automatically try to reach the target
        //there is telemetry in the periodic() method of the lift subsystem that will give us updates in the dashboard about the current ticks and current state
    }

    @Override
    public boolean isFinished() {

        //get the currentTicks from the subsystem because we need it to figure out if we have reached our target
        currentTicks = liftSlideSubsystem.getCurrentTicks();

        // Declare a boolean variable (e.g., finished)
        // Compare the currentTicks to the targetTicks to a threshold (LIFT_HEIGHT_TICK_THRESHOLD) and save as the boolean
        // For example, say our target is 2000 ticks and we are at 1997 - we would want that to count as being close enough
        boolean finished = Math.abs(currentTicks - targetTicks) <  LiftSlideSubsystem.liftSlideParameters.LIFT_HEIGHT_TICK_THRESHOLD;

        //write an if statement to change the currentState to the targetState and return true if the finished boolean is true
        if (finished){
            //if the command is finished, then change the current state to the target state and return true (meaning the command will stop running)
            liftSlideSubsystem.setCurrentState(targetState);
            return true;
        }

        //compare the elapsed time to a timeout threshold and if the elapsed time is greater than the threshold return true
        timeout = timeoutTimer.seconds() > TIMEOUT_TIME_SECONDS;
        if (timeout){
            return true;
        }

        //if the command isn't finished and the command isn't timed out then return false because the command should still run
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //write an if statement that tells the user the command didn't finish normally but instead timed out
        if (timeout){
            TelemetryPacket p = new TelemetryPacket();
            //Put the target state in the packet
            p.addLine("Slide Move Timeout");
            p.put("Timeout Timer", timeoutTimer.seconds());
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}
