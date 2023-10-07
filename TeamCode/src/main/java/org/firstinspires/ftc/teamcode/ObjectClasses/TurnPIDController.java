package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {
    private double m_lastDegreesLeftToTurn;
    double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    private double Kp=0;
    private double Ki=0;
    private double Kd=0;
    private double lastError=0;

    public double updatePID(double reference, double state) {
        double error = reference - state;
        integralSum +=error * timer.seconds();
        double derivative = (error-lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative*Kd) + (integralSum*Ki);
        return output;
    }
}



