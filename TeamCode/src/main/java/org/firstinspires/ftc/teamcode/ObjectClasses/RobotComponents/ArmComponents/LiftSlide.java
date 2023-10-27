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

@Config
public class LiftSlide {

    private static int LIFT_HEIGHT_TICK_THRESHOLD = 5;
    private static double STARTING_LIFT_POWER = .2;
    private static DcMotorEx lift;
    private static LinearOpMode activeOpMode;

    public static double p=5, i=0, d=0;
    public static double f = 0;
    public static int targetTicks = 0;
    public static int currentTicks = 0;
    public static double power = STARTING_LIFT_POWER;

    private PIDFCoefficients pidfCoefficients;

    public enum LiftHeights {
        HIGH (35.6),
        MID(25.7),
        LOW (16.2);

        private double inches;
        private int ticks;
        private LiftHeights(double inches) {
            this.inches = inches;
            this.ticks = (int) (inches * 50);
        }
    }
    static LiftHeights targetLiftHeight = LiftHeights.LOW;
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
        lift.setPower(power);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Action liftToHighHeight(){
        return new LiftToHighHeight();
    }

    public Action liftToLowHeight(){
        return new LiftToLowHeight();
    }

    public Action liftToMidHeight(){
        return new LiftToMidHeight();
    }

    public static class LiftToLowHeight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentTicks = lift.getCurrentPosition();
            targetLiftHeight = LiftHeights.LOW;
            targetTicks = LiftHeights.LOW.ticks;

            lift.setTargetPosition(targetTicks);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetryPacket.put("Target Lift Height", targetLiftHeight);
            telemetryPacket.put("Target Ticks", targetTicks);
            telemetryPacket.put("Current Ticks", currentTicks);
            if (Math.abs(currentTicks - targetTicks) < LIFT_HEIGHT_TICK_THRESHOLD)
            {
                return false;
            } else return true;
        }
    }

    public class LiftToMidHeight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentTicks = lift.getCurrentPosition();
            targetTicks = LiftHeights.MID.ticks;
            targetLiftHeight = LiftHeights.MID;
            lift.setTargetPosition(targetTicks);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetryPacket.put("Target Lift Height", targetLiftHeight);
            telemetryPacket.put("Target Ticks", targetTicks);
            telemetryPacket.put("Current Ticks", currentTicks);
            if (Math.abs(currentTicks - targetTicks) < LIFT_HEIGHT_TICK_THRESHOLD)
            {
                return false;
            } else return true;
        }
    }

    public class LiftToHighHeight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentTicks = lift.getCurrentPosition();
            targetTicks = LiftHeights.HIGH.ticks;
            targetLiftHeight = LiftHeights.HIGH;
            lift.setTargetPosition(targetTicks);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetryPacket.put("Current Lift Height", currentliftHeight);
            telemetryPacket.put("Target Lift Height", targetLiftHeight);
            telemetryPacket.put("Target Ticks", targetTicks);
            telemetryPacket.put("Current Ticks", currentTicks);
            if (Math.abs(currentTicks - targetTicks) < LIFT_HEIGHT_TICK_THRESHOLD)
            {
                return false;
            } else return true;
        }
    }
}