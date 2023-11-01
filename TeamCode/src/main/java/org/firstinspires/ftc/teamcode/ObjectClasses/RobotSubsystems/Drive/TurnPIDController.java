package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;


import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class TurnPIDController {
    private double m_lastDegreesLeftToTurn;
    double integralSum = 0;
    private double targetAngle;
    double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;

    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double feedforward;

    public TurnPIDController(double target, double p, double i, double d, double f) {
        targetAngle = target;
        Kp = p;
        Ki = i;
        Kd = d;
        feedforward = f;
    }

    public double update(double currentAngle) {
        //P Term
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) error -= 360;

        //I Term
        accumulatedError += error * timer.seconds();
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D Term
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;
        timer.reset();

        //motor power calculation
        double output = feedforward * Math.signum(error) + .9 * Math.tanh(
                (error * Kp) + (accumulatedError * Ki) + (slope * Kd));

        Robot.getInstance().getActiveOpMode().telemetry.addData("error", error);
        Robot.getInstance().getActiveOpMode().telemetry.update();

        return -output;
    }
}




