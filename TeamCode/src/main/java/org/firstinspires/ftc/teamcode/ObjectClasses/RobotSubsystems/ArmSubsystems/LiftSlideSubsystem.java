package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class LiftSlideSubsystem extends SubsystemBase {

    public static class LiftSlideParameters {
        public int LIFT_HEIGHT_TICK_THRESHOLD = 15;
        public double STARTING_LIFT_POWER = .6;
        public double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=30;
        public double POS_P=5;
    }

    public static LiftSlideParameters liftSlideParameters = new LiftSlideParameters();

    public enum LiftStates {
        HIGH (2200),
        MID(1700),
        LOW (1200),
        HOME (125);

        public int ticks;

        //From ChatGPT
        //C=π×d
        //diameter of spool is 1,
        //C=3.14159inch circumference
        //Gobilda 5203 motor outputs 8192 ticks for one full revolution of motor shaft
        // So ~2608 ticks per inch.
        //this did not work at all.

        LiftStates(int t) {
            this.ticks = t;
        }
    }

    public DcMotorEx liftSlide;
    public PIDFCoefficients pidfCoefficients;

    public LiftStates currentState;
    public int currentTicks;
    public double currentDistanceInches;
    FtcDashboard dash;
    public double power;
    TelemetryPacket telemetryPacket;

    /** Constructor **/
    public LiftSlideSubsystem(final HardwareMap hMap, final String name) {
        liftSlide = hMap.get(DcMotorEx.class, name);
    }

    public void init (){
        //THIS Direction works better because the string doesn't coil up in the other direction on the spool
        liftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P,liftSlideParameters.VEL_I,liftSlideParameters.VEL_D,liftSlideParameters.VEL_F);
        liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);
        liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = liftSlideParameters.STARTING_LIFT_POWER;
        liftSlide.setPower(power);
        currentState = LiftStates.HOME;
        currentTicks = LiftStates.HOME.ticks;
        currentDistanceInches = LiftStates.HOME.ticks;
        liftSlide.setTargetPosition(currentTicks);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dash = FtcDashboard.getInstance();
    }

    public void periodic(){
        liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P,liftSlideParameters.VEL_I,liftSlideParameters.VEL_D,liftSlideParameters.VEL_F);
        liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);
        liftSlide.setPower(liftSlideParameters.STARTING_LIFT_POWER);

        telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current LiftSlide State", currentState);
        telemetryPacket.put("Current Ticks", currentTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}