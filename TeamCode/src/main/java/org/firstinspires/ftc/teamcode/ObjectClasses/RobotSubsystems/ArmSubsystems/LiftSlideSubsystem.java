package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LiftSlideSubsystem extends SubsystemBase {

    public static class LiftSlideParameters {
        public int LIFT_HEIGHT_TICK_THRESHOLD = 5;
        public double STARTING_LIFT_POWER = .2;
        public double p=5, i=0, d=0, f=0;
    }

    public static LiftSlideParameters liftSlideParameters;

    public enum LiftStates {
        HIGH (20),
        MID(10),
        LOW (5),
        HOME (0);

        public double inches;
        public int ticks;
        LiftStates(double inches) {
            this.inches = inches;
            this.ticks = (int) (inches * 10);
        }
    }

    public DcMotorEx liftSlide;
    public PIDFCoefficients pidfCoefficients;

    public LiftStates currentState;
    public int currentTicks;
    public double currentDistanceInches;

    public double power;

    /** Constructor **/
    public LiftSlideSubsystem(final HardwareMap hMap, final String name) {
        liftSlide = hMap.get(DcMotorEx.class, name);
    }

    public void init (){
        liftSlideParameters = new LiftSlideParameters();
        liftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        pidfCoefficients = new  PIDFCoefficients(liftSlideParameters.p,liftSlideParameters.i,liftSlideParameters.d,liftSlideParameters.f);
        liftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = liftSlideParameters.STARTING_LIFT_POWER;
        liftSlide.setPower(power);
        currentState = LiftStates.HOME;
        currentTicks = LiftStates.HOME.ticks;
        currentDistanceInches = LiftStates.HOME.inches;
        liftSlide.setTargetPosition(currentTicks);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic(){

    }
}