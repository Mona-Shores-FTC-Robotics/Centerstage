package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;

import static java.lang.Math.signum;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;

public class PIDTurn extends CommandBase {

    private double FINISH_THRESHOLD_IN_DEGREES=2;

    private double targetAngle;
    double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError=0;
    private double lastTime=0;
    private double currentAngle;

    private double feedforward;

    private double Kp=0;
    private double Ki=0;
    private double Kd=0;

    public PIDTurn(double target, double p, double i, double d, double f){
        targetAngle = target;
        Kp = p;
        Ki= i;
        Kd = d;
        feedforward = f;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        currentAngle = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;

        //P Term
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error >180) error -= 360;

        //I Term
        accumulatedError += error * timer.seconds();
        if (Math.abs(error) < 1) {
            accumulatedError=0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D Term
        double slope = 0;
        if (lastTime >0){
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //motor power calcualtion
        double output = feedforward * Math.signum(error) + .2 * Math.tanh(
                    (error * Kp) + (accumulatedError*Ki) + (slope*Kd));

        Robot.getInstance().getActiveOpMode().telemetry.addData("error", error);
        Robot.getInstance().getActiveOpMode().telemetry.update();

        Robot.getInstance().getDriveSubsystem().mecanumDrive.mecanumDriveSpeedControl(0,0, -output);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(currentAngle-targetAngle) < FINISH_THRESHOLD_IN_DEGREES)
        {
            return true;
        } else return false;
    }
}



