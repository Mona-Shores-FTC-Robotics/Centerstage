package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberParameters  {

        public ClimberArmStates CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED;
        public WinchMotorStates WINCH_MOTOR_STARTING_STATE = WinchMotorStates.WOUND;

        public double CLIMBER_ARM_THRESHOLD = .03;
        public double STOWED_VALUE = .5;
        public double READY_VALUE = .8;
        public double END_GAME_TIME = 120;

        public static int WINCH_TICK_THRESHOLD = 30;
        public static double LET_OUT_POWER = 0;
        public static double PULL_IN_POWER = .6;
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
        WOUND (0),
        UNWOUND (500);

        public int ticks;
        WinchMotorStates(int t) {
            this.ticks = t;
        }
        void SetStateTicks(int t){
            this.ticks = t;
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

    private GamepadHandling gamepadHandling;

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
        winchMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentWinchMotorState = WinchMotorStates.WOUND;
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        power = climberParameters.LET_OUT_POWER;
        winchMotor.setPower(power);
        winchMotor.setVelocity(0);
    }

    public void periodic(){

//        if (MatchConfig.teleOpTimer.seconds() > climberParameters.END_GAME_TIME)
//        {
//
//            gamepadHandling.getOperatorGamepad().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                    .whenPressed(new ReadyClimberArmCommand(this, ClimberArmStates.READY))
//                    .whenReleased(new PullWinchInCommand(this, WinchMotorStates.WOUND));
//        }

        ClimberArmStates.READY.SetState(climberParameters.READY_VALUE);
        ClimberArmStates.STOWED.SetState(climberParameters.STOWED_VALUE);
    }
}
