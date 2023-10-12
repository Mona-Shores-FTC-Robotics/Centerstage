package org.firstinspires.ftc.teamcode.ObjectClasses.Constants;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.RobotConstants.RobotType.ROBOT_VISION_FAST_MOTORS;

public class MotorParameters {

    /** DECLARATION OF OUR MOTOR PARAMETERS **/

    public double DEFAULT_P;
    public double DEFAULT_D;
    public double DEFAULT_I;
    public double DEFAULT_F;

    public double P; // default = 10
    public double D; // default = 0
    public double I; // default = 3
    public double F; // default = 0

    // DriveTrain physical constants
    public double MAX_MOTOR_SPEED_RPS;
    public double TICKS_PER_REV;
    public double DRIVE_GEAR_REDUCTION = 1.0;
    public double WHEEL_DIAMETER_INCHES = 3.93701;
    public double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    /** DECLARATION OF ROADRUNNER MOTOR PARAMETERS **/
    // drive model parameters
    public double inPerTick;
    public double lateralInPerTick;
    public double trackWidthTicks;

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

        /** Set our motor parameters for faster drive motors **/

        //TODO test driving around in teleop and tweak these
        //We either use these or the roadrunner ones, not both
        DEFAULT_P = 5; // default = 10
        DEFAULT_D = 0; // default = 0
        DEFAULT_I = 0; // default = 3
        DEFAULT_F = 0; // default = 0

        P = DEFAULT_P; // default = 10
        D = DEFAULT_D; // default = 0
        I = DEFAULT_I; // default = 3
        F = DEFAULT_F; // default = 0

        // DriveTrain physical constants
        MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        TICKS_PER_REV = 384.5;
        DRIVE_GEAR_REDUCTION = 1.0;
        WHEEL_DIAMETER_INCHES = 3.93701;
        COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

        /** Set Roadrunner motor parameters for faster drive motors **/

        // drive model parameters
         inPerTick = 0.0317919075144509; //60.5\1903
         lateralInPerTick = 0.0325115144947169; // 60\1845.5
         trackWidthTicks = 631.8289216104534;

        // feedforward parameters (in tick units)
         kS = 0.9574546275336608;
         kV = 0.004264232249424524;
         kA = 0.00055;

        // path profile parameters (in inches)
        maxWheelVel = 25;
        minProfileAccel = -30;
        maxProfileAccel = 30;

        // turn profile parameters (in radians)
        maxAngVel = Math.PI; // shared with path
        maxAngAccel = Math.PI;

        // path controller gains
        axialGain = 12;
        lateralGain = 3;
        headingGain = 8; // shared with turn

        axialVelGain = 1;
        lateralVelGain = 1;
        headingVelGain = 1; // shared with turn
    }


    private void Params_Slow() {


        /** Set our motor parameters for slower drive motors **/

        DEFAULT_P = 11; // default = 10
        DEFAULT_D = 3; // default = 0
        DEFAULT_I = 0; // default = 3
        DEFAULT_F = 12; // default = 0

        P = DEFAULT_P; // default = 10
        D = DEFAULT_D; // default = 0
        I = DEFAULT_I; // default = 3
        F = DEFAULT_F; // default = 0

        // DriveTrain physical constants
        MAX_MOTOR_SPEED_RPS = 312.0 / 60.0;
        TICKS_PER_REV = 537.7;
        DRIVE_GEAR_REDUCTION = 1.0;
        WHEEL_DIAMETER_INCHES = 3.93701;
        COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

        /** Set Roadrunner motor parameters for slower drive motors **/

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

