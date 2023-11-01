package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftSlideSubsystem extends SubsystemBase {

    public static class LiftSlideParameters {
        public int LIFT_HEIGHT_TICK_THRESHOLD = 15;
        public double EXTENSION_LIFT_POWER = .6;
        public double RETRACTION_LIFT_POWER = .3;
        public double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=28;
        public double POS_P=5;
    }
    public static class LiftSlideHeights{
        public int HOME_HEIGHT_TICKS=5;
        public int LOW_HEIGHT_TICKS=800;
        public int MID_HEIGHT_TICKS=1700;
        public int HIGH_HEIGHT_TICKS=2400;
    }

    public int MAX_TARGET_TICKS = 2500;
    public int MIN_TARGET_TICKS = 0;

    public static LiftSlideParameters liftSlideParameters = new LiftSlideParameters();
    public static LiftSlideHeights liftSlideHeights = new LiftSlideHeights();

    public enum LiftStates {
        HIGH, MID, LOW, HOME;
        public int ticks;

        static {
            HIGH.ticks = liftSlideHeights.HIGH_HEIGHT_TICKS;
            MID.ticks = liftSlideHeights.MID_HEIGHT_TICKS;
            LOW.ticks = liftSlideHeights.LOW_HEIGHT_TICKS;
            HOME.ticks = liftSlideHeights.HOME_HEIGHT_TICKS;
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
    public void setCurrentState(LiftStates state) {
        this.currentState = state;
    }

    private int currentTicks;
    public int getCurrentTicks() {
        return currentTicks;
    }

    private double power;
    private FtcDashboard dash;
    private TelemetryPacket telemetryPacket;

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
        power = liftSlideParameters.EXTENSION_LIFT_POWER;
        liftSlide.setPower(power);

        currentState = LiftStates.HOME;
        currentTicks = LiftStates.HOME.ticks;
        liftSlide.setTargetPosition(currentTicks);
        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dash = FtcDashboard.getInstance();
    }

    public void periodic(){
        liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P,liftSlideParameters.VEL_I,liftSlideParameters.VEL_D,liftSlideParameters.VEL_F);
        liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);

        LiftStates.HOME.setLiftHeightTicks(liftSlideHeights.HOME_HEIGHT_TICKS);
        LiftStates.LOW.setLiftHeightTicks(liftSlideHeights.LOW_HEIGHT_TICKS);
        LiftStates.MID.setLiftHeightTicks(liftSlideHeights.MID_HEIGHT_TICKS);
        LiftStates.HIGH.setLiftHeightTicks(liftSlideHeights.HIGH_HEIGHT_TICKS);

        currentTicks = liftSlide.getCurrentPosition();

        telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current LiftSlide State", currentState);
        telemetryPacket.put("Current Ticks", currentTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}