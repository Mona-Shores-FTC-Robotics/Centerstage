package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem.ClimberParameters.LET_OUT_POWER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem.ClimberParameters.PULL_IN_POWER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem.ClimberParameters.WINCH_TICK_THRESHOLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PullWinchInCommand extends CommandBase {
    //Declare and set a timeout threshold for the command called TIMEOUT_TIME_SECONDS - I suggest 1.5 seconds for now
    private double TIMEOUT_TIME_SECONDS = 3;

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private ClimberSubsystem.WinchMotorStates targetState;
    //Declare currentTicks and targetTicks for use locally
    private int targetTicks;
    private int currentTicks;

    //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish
    private ElapsedTime timeoutTimer;

    //declare a timeout boolean
    boolean timeout;

    Telemetry telemetry;

    public PullWinchInCommand(ClimberSubsystem subsystem, ClimberSubsystem.WinchMotorStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
        timeoutTimer = new ElapsedTime();
        addRequirements(climberSubsystem);
    }
    @Override
    public void initialize() {
        //When the command is first run set the targetState of the subsystem to the targetState and set the target ticks to the target ticks of that state
        climberSubsystem.setTargetState(targetState);
        climberSubsystem.setTargetTicks(climberSubsystem.getTargetState().ticks);

        //reset the timer
        timeoutTimer.reset();
        //set the timeout to false since we have not timed out yet
        timeout=false;

        //get the currentTicks and the targetTicks from the subsystem
        currentTicks = climberSubsystem.getCurrentTicks();
        targetTicks = climberSubsystem.getTargetTicks();

        //Check if targetTicks is greater than MAX_TARGET_TICKS and if it is set the target to the max
        //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot
        if (targetTicks> climberSubsystem.MAX_TARGET_TICKS)
        {
            targetTicks=climberSubsystem.MAX_TARGET_TICKS;
        }

        //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
        //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot
        if (targetTicks < climberSubsystem.MIN_TARGET_TICKS)
        {
            targetTicks=climberSubsystem.MIN_TARGET_TICKS;
        }

        //if the target ticks are higher than the current ticks, then use EXTENSION_POWER
        if (targetTicks > currentTicks) {
            climberSubsystem.winchMotor.setPower(PULL_IN_POWER);
        }

        //if the target ticks are lower than the current ticks, then use RETRACTION_POWER
        if (targetTicks < currentTicks) {
            climberSubsystem.winchMotor.setPower(LET_OUT_POWER);
        }

        //Set the target position using the targetTicks
        climberSubsystem.winchMotor.setTargetPosition(targetTicks);

        //set the lift motor to RUN TO POSITION - this might not be necessary
        climberSubsystem.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void execute() {

    }
    @Override
    public boolean isFinished() {
        // Declare a boolean variable (e.g., finished)
        // Compare the currentTicks to the targetTicks to a threshold (LIFT_HEIGHT_TICK_THRESHOLD) and save as the boolean
        // For example, say our target is 2000 ticks and we are at 1997 - we would want that to count as being close enough
        boolean finished = Math.abs(climberSubsystem.getCurrentTicks() - climberSubsystem.getTargetTicks()) <  WINCH_TICK_THRESHOLD;

        //write an if statement to change the currentState to the targetState and return true if the finished boolean is true
        if (finished){
            //if the command is finished, then return true (meaning the command will stop running once it runs end() one time)
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
        TelemetryPacket p = new TelemetryPacket();
        //write an if statement that tells the user the command didn't finish normally but instead timed out
        if (!timeout && !interrupted)
        {
            //Report the command finished
            p.addLine("LiftSlide Move COMPLETE From " + climberSubsystem.getCurrentState() + " to " + climberSubsystem.getTargetState() + " in " + String.format("%.2f", timeoutTimer.seconds()) + " seconds");
            //change the current state to the target state
            climberSubsystem.setCurrentState(climberSubsystem.getTargetState());
        }
        if (timeout){
            //Put the target state in the packet
            p.addLine("LiftSlide Move TIMEOUT");
            p.put("Timeout Timer", timeoutTimer.seconds());
        }
        if (interrupted){
            //Put the target state in the packet
            p.addLine("LiftSlide Move INTERRUPTED From " + climberSubsystem.getCurrentState() + " to " + climberSubsystem.getTargetState() + " at " + timeoutTimer.seconds() + " seconds");
        }

        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}