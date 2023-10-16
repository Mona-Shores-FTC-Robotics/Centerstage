package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class LiftSlide {
    public DcMotorEx lift;
    LinearOpMode activeOpMode;

    public static double p=5, i=0, d=0;
    public static double f = 0;
    public static int targetTicks = 0;
    public static double power = 1;

    private PIDFCoefficients pidfCoefficients;

//    private double MOTOR_SPEED_RPM = 435;
//    public static double TICKS_PER_REV = 	((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0); //384.5
//    private double MOTOR_SPEED_TPS = MOTOR_SPEED_RPM * TICKS_PER_REV / 60.0;
//    private static double INCHES_PER_REV = 18.0 * 3.0 * 3.0 / 25.4; // pulley teeth * tooth spacing * lift stages / (mm/inch)
//    public static double TICKS_PER_INCH = TICKS_PER_REV/INCHES_PER_REV;

    public enum LiftHeights {
        HIGH (35.6),
        MEDIUM (25.7),
        LOW (16.2);

        private double inches;
        private int ticks;
        private LiftHeights(double inches) {
            this.inches = inches;
            this.ticks = (int) (inches * 50);
        }
    }
    LiftHeights targetLiftHeight = LiftHeights.LOW;
    LiftHeights currentliftHeight = LiftHeights.LOW;

    /* Constructor     */
    public LiftSlide() {

    }

    /* Initialization */
    public void init (){
        activeOpMode = Robot.getInstance().getActiveOpMode();
        //TODO change this to the name of the lift slide motor
        lift = Robot.getInstance().getHardwareMap().get(DcMotorEx.class, "LBDrive");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        pidfCoefficients = new  PIDFCoefficients(p,i,d,f);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        lift.setPower(power);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        lift.setTargetPosition(targetTicks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FtcDashboard.getInstance().getTelemetry().addData("Target", targetTicks);
        FtcDashboard.getInstance().getTelemetry().addData("Current Position", lift.getCurrentPosition());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public void LiftToPresetHeight(LiftHeights targetHeight){
        // Set the targetPosition to the ticks associated with the target height (LOW/MEDIUM/HIGH)
        targetTicks = targetHeight.ticks;
        lift.setTargetPosition(targetTicks);
    }
}