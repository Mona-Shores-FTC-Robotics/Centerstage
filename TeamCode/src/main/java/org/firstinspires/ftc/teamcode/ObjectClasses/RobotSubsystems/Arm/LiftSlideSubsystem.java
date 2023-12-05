package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideHeights.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import android.service.autofill.FieldClassification;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class LiftSlideSubsystem extends SubsystemBase {


    public static class LiftSlideParameters {
        public int LIFT_HEIGHT_TICK_THRESHOLD = 45;
        public double TIMEOUT_TIME_SECONDS = 2;
        public double EXTENSION_LIFT_POWER = .6;
        public double RETRACTION_LIFT_POWER = .33;
        public double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=48;
        public double VEL_P_DOWN=7, VEL_I_DOWN=0, VEL_D_DOWN=0, VEL_F_DOWN=15;
        public double POS_P=9;
        public double POS_P_DOWN=12;
        public double SCALE_FACTOR_FOR_MANUAL_LIFT=150;
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 50;
        public double SAFE_ZONE_FOR_MANUAL_LIFT = 950;
    }

    public static class LiftSlideHeights{
        public int ZERO_HEIGHT_TICKS=0;
        public int HOME_HEIGHT_TICKS=0;
        public int SAFE_HEIGHT_TICKS=25;
        public int AUTO_LOW_HEIGHT_TICKS=900;
        public int AUTO_MID_HEIGHT_TICKS=1250;
        public int AUTO_HIGH_HEIGHT_TICKS=1700;

        public int LOW_HEIGHT_TICKS=1365;
        public int MID_HEIGHT_TICKS=2100;
        public int HIGH_HEIGHT_TICKS=2800;
        public int MAX_HEIGHT_TICKETS=2800;
    }

    public final int MAX_TARGET_TICKS = 2800;
    public final int MIN_TARGET_TICKS = 0;

    public static LiftSlideSubsystem.LiftSlideParameters liftSlideParameters = new LiftSlideParameters();
    public static LiftSlideSubsystem.LiftSlideHeights liftSlideHeights = new LiftSlideHeights();

    public enum LiftStates {
        AUTO_LOW, AUTO_MID, AUTO_HIGH, MAX, HIGH, MID, LOW, SAFE, HOME, ZERO, MANUAL;
        public int ticks;

        static {

            MAX.ticks = liftSlideHeights.MAX_HEIGHT_TICKETS;
            HIGH.ticks = liftSlideHeights.HIGH_HEIGHT_TICKS;
            MID.ticks = liftSlideHeights.MID_HEIGHT_TICKS;;
            LOW.ticks = liftSlideHeights.LOW_HEIGHT_TICKS;;
            HOME.ticks = liftSlideHeights.HOME_HEIGHT_TICKS;;
            ZERO.ticks = liftSlideHeights.ZERO_HEIGHT_TICKS;;
            SAFE.ticks = liftSlideHeights.SAFE_HEIGHT_TICKS;;
            AUTO_LOW.ticks = liftSlideHeights.AUTO_LOW_HEIGHT_TICKS;
            AUTO_MID.ticks = liftSlideHeights.AUTO_MID_HEIGHT_TICKS;
            AUTO_HIGH.ticks = liftSlideHeights.AUTO_HIGH_HEIGHT_TICKS;
        }
        public void setLiftHeightTicks(int t){
            this.ticks = t;
        }
    }

    public DcMotorEx liftSlide;

    private LiftStates currentState;
    public void setCurrentState(LiftStates state) {currentState = state;}
    public LiftStates getCurrentState() {return currentState;}

    private LiftStates targetState;
    public void setTargetState(LiftStates state) {targetState = state;}
    public LiftStates getTargetState() {return targetState;}

    private int targetTicks;
    public int getTargetTicks() {return targetTicks;}
    public void setTargetTicks(int ticks) {targetTicks=ticks;}

    private int currentTicks;
    public int getCurrentTicks() {
        return currentTicks;
    }
    public LiftSlideSubsystem.LiftStates currentDeliverHeight = LiftSlideSubsystem.LiftStates.LOW;


    public LiftSlideSubsystem.LiftStates getDeliverHeight() {
        return currentDeliverHeight;
    }


    public void setDeliverHeight(LiftSlideSubsystem.LiftStates targetLiftState) {
        currentDeliverHeight = targetLiftState;
    }



    private double power;

    public boolean isStageOneReleasePixels() {
        return stageOneReleasePixels;
    }

    public void setStageOneReleasePixels(boolean stageOneReleasePixels) {
        this.stageOneReleasePixels = stageOneReleasePixels;
    }

    private boolean stageOneReleasePixels = true;


    public LiftSlideSubsystem(final HardwareMap hMap, final String name) {
        liftSlide = hMap.get(DcMotorEx.class, name);
    }

    public void init (){
        //This Direction works better because the string doesn't coil up in the other direction on the spool
        liftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P, liftSlideParameters.VEL_I, liftSlideParameters.VEL_D, liftSlideParameters.VEL_F);
        liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);
        liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = liftSlideParameters.EXTENSION_LIFT_POWER;
        liftSlide.setPower(power);

        currentState = LiftStates.ZERO;
        liftSlide.setTargetPosition(currentState.ticks);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic(){


        LiftStates.HOME.setLiftHeightTicks(liftSlideHeights.HOME_HEIGHT_TICKS);
        LiftStates.SAFE.setLiftHeightTicks(liftSlideHeights.SAFE_HEIGHT_TICKS);
        LiftStates.LOW.setLiftHeightTicks(liftSlideHeights.LOW_HEIGHT_TICKS);
        LiftStates.MID.setLiftHeightTicks(liftSlideHeights.MID_HEIGHT_TICKS);
        LiftStates.HIGH.setLiftHeightTicks(liftSlideHeights.HIGH_HEIGHT_TICKS);
        LiftStates.MAX.setLiftHeightTicks(liftSlideHeights.MAX_HEIGHT_TICKETS);

        //this is the one call per loop to get the currentPosition from the lift motor
        currentTicks = liftSlide.getCurrentPosition();

        MatchConfig.telemetryPacket.put("LiftSlide State", currentState);
        MatchConfig.telemetryPacket.put("LiftSlide Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("LiftSlide Deliver Height", Robot.getInstance().getLiftSlideSubsystem().getDeliverHeight());

        if (targetState!=currentState) {
            MatchConfig.telemetryPacket.put("LiftSlide Target State", targetState);
            MatchConfig.telemetryPacket.put("LiftSlide Target Ticks", targetTicks);
        }
    }
}