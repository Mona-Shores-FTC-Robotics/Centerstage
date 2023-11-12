package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class RotateShoulderCommand extends CommandBase {
    private double TIMEOUT_TIME_MILLISECONDS = 800;
    // The subsystem the command runs on
    private final ShoulderSubsystem shoulderSubsystem;

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;
    private double targetPosition;
    TelemetryPacket telemetryPacket;
    //declare a timeout boolean
    boolean timeout;

    ElapsedTime shoulderTimer = new ElapsedTime();

    public RotateShoulderCommand(ShoulderSubsystem subsystem, ShoulderSubsystem.ShoulderStates inputState) {
            shoulderSubsystem = subsystem;
            targetState = inputState;
            targetPosition = targetState.position;

            //require this subsystem
            addRequirements(shoulderSubsystem);
        }

    @Override
    public void initialize() {
        shoulderSubsystem.setTargetState(targetState);
        shoulderSubsystem.shoulder.setPosition(targetState.position);
        //create a new telemetry packet for this command
        shoulderTimer.reset();
        timeout=false;
    }

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean done = shoulderTimer.milliseconds() > ShoulderSubsystem.shoulderParameters.SHOULDER_ROTATE_THRESHOLD_MILLISECONDS;
            if (done)
            {
                return true;
            }

        timeout = shoulderTimer.milliseconds() > TIMEOUT_TIME_MILLISECONDS;
        if (timeout){
            return true;
        }

            return false;
    }


    @Override
    public void end(boolean interrupted) {
        TelemetryPacket p = new TelemetryPacket();
        //write an if statement that tells the user the command didn't finish normally but instead timed out
        if (!timeout && !interrupted)
        {
            //Report the command finished
            p.addLine("Shoulder Move COMPLETE From " + shoulderSubsystem.currentState + " to " + shoulderSubsystem.targetState + " in " + String.format("%.2f", shoulderTimer.milliseconds()) + " milliseconds");
            //change the current state to the target state
            shoulderSubsystem.setCurrentState(shoulderSubsystem.targetState);
        }
        if (timeout){
            //Put the target state in the packet
            p.addLine("Shoulder TIMEOUT");
            p.put("Timeout Timer", shoulderTimer.milliseconds());
        }
        if (interrupted){
            //Put the target state in the packet
            p.addLine("Shoulder Move INTERRUPTED From " + shoulderSubsystem.currentState + " to " + shoulderSubsystem.targetState + " at " + shoulderTimer.milliseconds() + " milliseconds");
        }

        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

}
