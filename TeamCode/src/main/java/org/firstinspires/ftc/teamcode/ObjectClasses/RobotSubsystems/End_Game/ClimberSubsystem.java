package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;

@Config
public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberParameters  {

        public ClimberArmStates CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
        public WinchMotorStates WINCH_MOTOR_STARTING_STATE = WinchMotorStates.ROBOT_DOWN;

        public double CLIMBER_ARM_THRESHOLD = .03;
        public double STOWED_VALUE = .5;
        public double READY_VALUE = .7;
        public double END_GAME_TIME = 120;

        public static int WINCH_TICK_THRESHOLD = 30;
        public static double ROBOT_DOWN_POWER = -.8;
        public static double ROBOT_UP_POWER = .8;
        public static double VEL_P=5, VEL_I=0, VEL_D=0, VEL_F=38;
        public static double POS_P=5;
        public static double SCALE_FACTOR_FOR_WINCH=150;
    }

    public final int MAX_TARGET_TICKS = 400;
    public final int MIN_TARGET_TICKS = 0;

    public static ClimberParameters climberParameters = new ClimberParameters();

    public enum ClimberArmStates {
        STOWED (.5),
        READY (.8);
        public double position;
        ClimberArmStates(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public enum WinchMotorStates {
        ROBOT_DOWN(-.3),
        ROBOT_UP(1),
        OFF(0);

        public double power;
        WinchMotorStates(double p) {
            this.power = p;
        }
    }

    public Servo climberArm;
    public DcMotorEx winchMotor;
    public ClimberSubsystem.ClimberArmStates currentClimberArmState;
    public double currentClimberArmPosition;
    public ClimberSubsystem.WinchMotorStates currentWinchMotorState;
    public double power;


    ClimberSubsystem.WinchMotorStates currentState;
    public void setCurrentState(ClimberSubsystem.WinchMotorStates state) {currentState = state;}
    public ClimberSubsystem.WinchMotorStates getCurrentState() {return currentState;}

    private ClimberSubsystem.WinchMotorStates targetState;
    public void setTargetState(ClimberSubsystem.WinchMotorStates state) {targetState = state;}
    public ClimberSubsystem.WinchMotorStates getTargetState() {return targetState;}

    private int targetTicks;
    public int getTargetTicks() {return targetTicks;}
    public void setTargetTicks(int ticks) {targetTicks=ticks;}

    private int currentTicks;
    public int getCurrentTicks() {
        return currentTicks;
    }


    public ClimberSubsystem(final HardwareMap hMap, final String climberArmName, final String winchMotorName) {
        climberArm = hMap.servo.get(climberArmName);
        winchMotor = hMap.get(DcMotorEx.class, winchMotorName);
    }

    public void init() {
        winchMotorInit();
        climberArmInit();
    }

    private void climberArmInit() {
        ClimberArmStates.READY.SetState(climberParameters.READY_VALUE);
        ClimberArmStates.STOWED.SetState(climberParameters.STOWED_VALUE);

        currentClimberArmState = climberParameters.CLIMBER_ARM_STARTING_STATE;
        currentClimberArmPosition = currentClimberArmState.position;
    }

    private void winchMotorInit() {
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentWinchMotorState = WinchMotorStates.OFF;
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setPower(0);
        winchMotor.setVelocity(0);
    }

    public void periodic(){
        ClimberArmStates.READY.SetState(climberParameters.READY_VALUE);
        ClimberArmStates.STOWED.SetState(climberParameters.STOWED_VALUE);
    }
}
