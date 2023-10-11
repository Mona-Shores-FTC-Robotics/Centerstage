package org.firstinspires.ftc.teamcode.ObjectClasses.Constants;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.RobotConstants.RobotType.ROBOT_VISION_FAST_MOTORS;

public class MotorParameters {
    // drive model parameters
    public static double inPerTick;
    public static double lateralInPerTick;
    public static double trackWidthTicks;

    // feedforward parameters (in tick units)
    public double kS;
    public double kV;
    public double kA;

    // path profile parameters (in inches)
    public double maxWheelVel;
    public double minProfileAccel;
    public double maxProfileAccel;

    // turn profile parameters (in radians)
    public double maxAngVel; // shared with path
    public double maxAngAccel;

    // path controller gains
    public double axialGain;
    public double lateralGain;
    public double headingGain; // shared with turn

    public double axialVelGain;
    public double lateralVelGain;
    public double headingVelGain; // shared with turn

    public void init() {
        if (RobotConstants.getRobot() == ROBOT_VISION_FAST_MOTORS) {
            Params_Fast();
        } else
        {
            Params_Slow();
        }
    }

    private void Params_Fast() {
        // drive model parameters
        inPerTick = 0.0317919075144509; //60.5\1903
        lateralInPerTick = 0.0325115144947169; // 60\1845.5
        trackWidthTicks = 893.5920803662788;

        // feedforward parameters (in tick units)
        kS = 0.7703864947833408;
        kV = 0.00436466666183017;
        kA = 0.00055;

        // path profile parameters (in inches)
        maxWheelVel = 25;
        minProfileAccel = -30;
        maxProfileAccel = 30;

        // turn profile parameters (in radians)
        maxAngVel = Math.PI; // shared with path
        maxAngAccel = Math.PI;

        // path controller gains
        axialGain = 8;
        lateralGain = 8;
        headingGain = 4; // shared with turn

        axialVelGain = .5;
        lateralVelGain = .5;
        headingVelGain = .5; // shared with turn
    }

    private void Params_Slow() {
        // drive model parameters
        inPerTick = 0.022365950344252; // 90.8in-37.2 2396.5ticks
        lateralInPerTick = 0.0280188186095139; //1913
        trackWidthTicks = 893.5920803662788;

        // feedforward parameters (in tick units)
        kS = 0.7703864947833408;
        kV = 0.00436466666183017;
        kA = 0.00055;

        // path profile parameters (in inches)
        maxWheelVel = 25;
        minProfileAccel = -30;
        maxProfileAccel = 30;

        // turn profile parameters (in radians)
        maxAngVel = Math.PI; // shared with path
        maxAngAccel = Math.PI;

        // path controller gains
        axialGain = 8;
        lateralGain = 8;
        headingGain = 4; // shared with turn

        axialVelGain = .5;
        lateralVelGain = .5;
        headingVelGain = .5; // shared with turn
    }
}

