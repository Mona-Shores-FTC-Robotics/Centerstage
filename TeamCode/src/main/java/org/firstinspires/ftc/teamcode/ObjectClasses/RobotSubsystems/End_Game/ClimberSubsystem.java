package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.PurplePixelPusher.PixelPusherSubsystem;

@Config
public class ClimberSubsystem extends SubsystemBase {

    public static class ClimberParameters  {

        public ClimberArmStates CLIMBER_ARM_STARTING_STATE = ClimberArmStates.STOWED_STEP3;
        public WinchMotorStates WINCH_MOTOR_STARTING_STATE = WinchMotorStates.ROBOT_DOWN;

        public double STOWED_STEP1_VALUE = .6;
        public double STOWED_STEP2_VALUE = .53;
        public double STOWED_STEP3_VALUE = .5;
        public double READY_VALUE = .85;

        public double ROBOT_DOWN_POWER = -.8;
        public double ROBOT_UP_POWER = .8;
    }

    //.6 -> .53 -> .5 add time between

    public static ClimberParameters climberParameters = new ClimberParameters();

    public enum ClimberArmStates {
        STOWED_STEP1,
        STOWED_STEP2,
        STOWED_STEP3,
        READY;

        public double position;
        static {
            STOWED_STEP1.position = climberParameters.STOWED_STEP1_VALUE;
            STOWED_STEP2.position = climberParameters.STOWED_STEP2_VALUE;
            STOWED_STEP3.position = climberParameters.STOWED_STEP3_VALUE;
            READY.position = climberParameters.READY_VALUE;
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
        void SetState(double p){
            this.power = p;
        }
    }

    public Servo climberArm;
    public DcMotorEx winchMotor;
    public ClimberSubsystem.ClimberArmStates currentClimberArmState;
    public ClimberSubsystem.WinchMotorStates currentWinchMotorState;

    public void setCurrentWinchMotorState(ClimberSubsystem.WinchMotorStates state) {currentWinchMotorState = state;}
    public ClimberSubsystem.WinchMotorStates getCurrentWinchMotorState() {return currentWinchMotorState;}

    public void setCurrentClimberArmState(ClimberSubsystem.ClimberArmStates state) {currentClimberArmState = state;}
    public ClimberSubsystem.ClimberArmStates getCurrentClimberArmState() {return currentClimberArmState;}

    public ClimberSubsystem(final HardwareMap hMap, final String climberArmName, final String winchMotorName) {
        climberArm = hMap.servo.get(climberArmName);
        winchMotor = hMap.get(DcMotorEx.class, winchMotorName);
    }

    public void init() {
        winchMotorInit();
        climberArmInit();
    }

    private void climberArmInit() {
        SetClimberArmStatesValues();
        currentClimberArmState = climberParameters.CLIMBER_ARM_STARTING_STATE;
    }

    private void winchMotorInit() {
        SetWinchStatesValues();
        currentWinchMotorState = climberParameters.WINCH_MOTOR_STARTING_STATE;
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentWinchMotorState = WinchMotorStates.OFF;
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setPower(0);
        winchMotor.setVelocity(0);
    }

    public void periodic(){
        //Save the values to the enum every loop so we can adjust in the dashboard
        SetClimberArmStatesValues();
        SetWinchStatesValues();

        //Add the Winch Motor State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("End Game Winch State", currentWinchMotorState);

        //Add the Climber Arm State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Climber Arm State", currentClimberArmState);
    }

    private void SetWinchStatesValues() {
        WinchMotorStates.ROBOT_DOWN.SetState(climberParameters.ROBOT_DOWN_POWER);
        WinchMotorStates.ROBOT_UP.SetState(climberParameters.ROBOT_UP_POWER);
    }

    private void SetClimberArmStatesValues() {
        ClimberArmStates.READY.SetState(climberParameters.READY_VALUE);
        ClimberArmStates.STOWED_STEP1.SetState(climberParameters.STOWED_STEP1_VALUE);
        ClimberArmStates.STOWED_STEP2.SetState(climberParameters.STOWED_STEP2_VALUE);
        ClimberArmStates.STOWED_STEP3.SetState(climberParameters.STOWED_STEP3_VALUE);
    }


}

