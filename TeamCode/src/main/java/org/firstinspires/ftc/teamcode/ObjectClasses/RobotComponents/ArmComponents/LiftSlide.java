package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

//todo Write the code for an action to move the lift

@Config
public class LiftSlide {

    private static int LIFT_HEIGHT_TICK_THRESHOLD = 20;
    private static double STARTING_LIFT_POWER = .5;
    private static int HIGH_HEIGHT = 24;
    private static int MID_HEIGHT = 12;
    private static int LOW_HEIGHT = 0;
    public static double p=5, i=0, d=0;
    public static double f = 0;

    private static DcMotorEx lift;
    private static LinearOpMode activeOpMode;
    private static int targetTicks = 0;
    private static int currentTicks = 0;
    private static double power = STARTING_LIFT_POWER;

    private static PIDFCoefficients pidfCoefficients;
    private static LiftSlideStates targetState = LiftSlideStates.LOW;
    private static LiftSlideStates currentState = LiftSlideStates.LOW;

    public enum LiftSlideStates {
        HIGH (HIGH_HEIGHT),
        MID(MID_HEIGHT),
        LOW (LOW_HEIGHT);

        private double inches;
        private int ticks;
        private LiftSlideStates(double inches) {
            this.inches = inches;
            this.ticks = (int) (inches * 50);
        }
    }


    /* Constructor     */
    public LiftSlide() {

    }

    /* Initialization */
    public void init (){
        activeOpMode = Robot.getInstance().getActiveOpMode();
        lift = Robot.getInstance().getHardwareMap().get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        pidfCoefficients = new  PIDFCoefficients(p,i,d,f);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(power);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Action moveLift(LiftSlideStates s){
        return new LiftSlide.MoveLift(s);
    }

    public static class MoveLift implements Action{

        public MoveLift(LiftSlideStates s) {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentTicks = lift.getCurrentPosition();
            targetState = LiftSlideStates.LOW;
            targetTicks = LiftSlideStates.LOW.ticks;

            lift.setTargetPosition(targetTicks);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetryPacket.put("Target Lift Height", targetState);
            telemetryPacket.put("Target Ticks", targetTicks);
            telemetryPacket.put("Current Ticks", currentTicks);
            if (Math.abs(currentTicks - targetTicks) < LIFT_HEIGHT_TICK_THRESHOLD)
            {
                return true;
            } else return false;
        }
    }
}