package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideHeights.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSlideSubsystem extends SubsystemBase {

    @Config
    public static class LiftSlideParameters {
        public static int LIFT_HEIGHT_TICK_THRESHOLD = 110;
        public static double EXTENSION_LIFT_POWER = .6;
        public static double RETRACTION_LIFT_POWER = .3;
        public static double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=38;
        public static double POS_P=5;
        public static double SCALE_FACTOR_FOR_MANUAL_LIFT=150;
        public static double LIFT_DEAD_ZONE;
    }

    @Config
    public static class LiftSlideHeights{
        public static int ZERO_HEIGHT_TICKS=0;
        public static int HOME_HEIGHT_TICKS=25;
        public static int SAFE_HEIGHT_TICKS=150;
        public static int LOW_HEIGHT_TICKS=1200;
        public static int MID_HEIGHT_TICKS=2200;
        public static int HIGH_HEIGHT_TICKS=2200;
    }

    public final int MAX_TARGET_TICKS = 2500;
    public final int MIN_TARGET_TICKS = 0;

    public enum LiftStates {
        HIGH, MID, LOW, SAFE, HOME, ZERO, MANUAL;
        public int ticks;

        static {
            HIGH.ticks = HIGH_HEIGHT_TICKS;
            MID.ticks = MID_HEIGHT_TICKS;
            LOW.ticks = LOW_HEIGHT_TICKS;
            HOME.ticks = HOME_HEIGHT_TICKS;
            ZERO.ticks = ZERO_HEIGHT_TICKS;
            SAFE.ticks = SAFE_HEIGHT_TICKS;
        }

        public void setLiftHeightTicks(int t){
            this.ticks = t;
        }

        //From ChatGPT
        //C=π×d
        //diameter of spool is 1,
        //C=3.14159inch circumference
        //Gobilda 5203 motor outputs 8192 ticks for one full revolution of motor shaft
        // So ~2608 ticks per inch.
        //this did not work at all - why not?
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

    private double power;
    private TelemetryPacket telemetryPacket;

    /** Constructor **/
    public LiftSlideSubsystem(final HardwareMap hMap, final String name) {
        liftSlide = hMap.get(DcMotorEx.class, name);
    }

    public void init (){
        //THIS Direction works better because the string doesn't coil up in the other direction on the spool
        liftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftSlide.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
        liftSlide.setPositionPIDFCoefficients(POS_P);
        liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = EXTENSION_LIFT_POWER;
        liftSlide.setPower(power);

        currentState = LiftStates.ZERO;
        currentTicks = LiftStates.ZERO.ticks;
        liftSlide.setTargetPosition(currentTicks);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic(){
        liftSlide.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
        liftSlide.setPositionPIDFCoefficients(POS_P);

        LiftStates.HOME.setLiftHeightTicks(HOME_HEIGHT_TICKS);
        LiftStates.SAFE.setLiftHeightTicks(SAFE_HEIGHT_TICKS);
        LiftStates.LOW.setLiftHeightTicks(LOW_HEIGHT_TICKS);
        LiftStates.MID.setLiftHeightTicks(MID_HEIGHT_TICKS);
        LiftStates.HIGH.setLiftHeightTicks(HIGH_HEIGHT_TICKS);

        //this is the one call per loop to get the currentPosition from the lift motor
        currentTicks = liftSlide.getCurrentPosition();

        telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current LiftSlide State", currentState);
        telemetryPacket.put("Current Ticks", currentTicks);

        if (targetState!=currentState) {
            telemetryPacket.put("Target LiftSlide State", targetState);
            telemetryPacket.put("Target Ticks", targetTicks);
        }

        //send the packet to the dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}