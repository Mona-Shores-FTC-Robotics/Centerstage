package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class LiftSlideSubsystem extends SubsystemBase {

    private static int LIFT_HEIGHT_TICK_THRESHOLD = 5;
    private static double STARTING_LIFT_POWER = .2;
    private static DcMotorEx liftSlide;
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
    public LiftSlideSubsystem(final HardwareMap hMap, final String name) {
        liftSlide = hMap.get(DcMotorEx.class, name);
    }

    /* Initialization */
    public void init (){
        liftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        pidfCoefficients = new  PIDFCoefficients(p,i,d,f);
        liftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftSlide.setPower(power);
        liftSlide.setTargetPosition(0);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic(){

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
            currentTicks = liftSlide.getCurrentPosition();
            targetLiftHeight = LiftHeights.LOW;
            targetTicks = LiftHeights.LOW.ticks;

            liftSlide.setTargetPosition(targetTicks);

            liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            currentTicks = liftSlide.getCurrentPosition();
            targetTicks = LiftHeights.MID.ticks;
            targetLiftHeight = LiftHeights.MID;
            liftSlide.setTargetPosition(targetTicks);
            liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            currentTicks = liftSlide.getCurrentPosition();
            targetTicks = LiftHeights.HIGH.ticks;
            targetLiftHeight = LiftHeights.HIGH;
            liftSlide.setTargetPosition(targetTicks);
            liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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