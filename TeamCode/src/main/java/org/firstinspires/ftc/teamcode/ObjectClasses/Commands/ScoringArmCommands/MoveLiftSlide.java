package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;

public class MoveLiftSlide extends CommandBase {

    // The subsystem the command runs on
    private final LiftSlideSubsystem liftSlideSubsystem;

    //declare target state & position
    private LiftSlideSubsystem.LiftStates targetState;
    private int targetTicks;

    Telemetry telemetry;

    public MoveLiftSlide(LiftSlideSubsystem subsystem, LiftSlideSubsystem.LiftStates inputState) {
        liftSlideSubsystem = subsystem;
        targetState = inputState;
        targetTicks = targetState.ticks;

        //add the subsystem to the requirements
        addRequirements(liftSlideSubsystem);
    }

    @Override
    public void initialize() {
        liftSlideSubsystem.liftSlide.setTargetPosition(targetTicks);
        liftSlideSubsystem.liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //create a new telemetry packet for this command
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;

    }

    public void execute() {
        liftSlideSubsystem.currentTicks = liftSlideSubsystem.liftSlide.getCurrentPosition();

        TelemetryPacket p = new TelemetryPacket();
        p.put("Target LiftSlide State", targetState);
        p.put("Target Ticks", targetTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Math.abs(liftSlideSubsystem.currentTicks - targetTicks) <  LiftSlideSubsystem.liftSlideParameters.LIFT_HEIGHT_TICK_THRESHOLD;
        if (finished){
            liftSlideSubsystem.currentState = targetState;
            return true;
        } else return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            //what should we do if interrupted?
        }
    }
}
