package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public static class IntakeParameters {
        public double INTAKE_ON_POWER = 1;
        public double INTAKE_REVERSE_POWER = -1;
        public double INTAKE_SLOW_POWER = .4;
        public double INTAKE_SLOW_REVERSE_POWER = -.4;
        public double INTAKE_SUPER_SLOW_POWER = .2;
        public double INTAKE_SUPER_SLOW_REVERSE_POWER = -.4;
        public double INTAKE_OFF_POWER = 0;
    }
    public static IntakeParameters intakeParameters = new IntakeParameters();

    public enum IntakeStates {
        INTAKE_OFF,
        INTAKE_ON,
        INTAKE_REVERSE,
        INTAKE_SLOW,
        INTAKE_SLOW_REVERSE,
        INTAKE_SUPER_SLOW,
        INTAKE_SUPER_SLOW_REVERSE;

        static {
            INTAKE_OFF.power = intakeParameters.INTAKE_OFF_POWER;
            INTAKE_ON.power = intakeParameters.INTAKE_ON_POWER;
            INTAKE_REVERSE.power = intakeParameters.INTAKE_REVERSE_POWER;
            INTAKE_SLOW.power = intakeParameters.INTAKE_SLOW_POWER;
            INTAKE_SLOW_REVERSE.power = intakeParameters.INTAKE_SLOW_REVERSE_POWER;
            INTAKE_SUPER_SLOW.power = intakeParameters.INTAKE_SUPER_SLOW_POWER;
            INTAKE_SUPER_SLOW_REVERSE.power = intakeParameters.INTAKE_SUPER_SLOW_REVERSE_POWER;
        }

        public double power;

        void SetStatePower(double pow){
            this.power = pow;
        }
    }

    public DcMotorEx intake1;
    public DcMotorEx intake2;

    public void setCurrentIntake1State(IntakeStates currentIntake1State) {
        this.currentIntake1State = currentIntake1State;
    }

    public void setCurrentIntake2State(IntakeStates currentIntake2State) {
        this.currentIntake2State = currentIntake2State;
    }

    public IntakeStates currentIntake1State;
    public IntakeStates currentIntake2State;

    public IntakeSubsystem(final HardwareMap hMap, final String name, final String name2){
        intake1 = hMap.get(DcMotorEx.class, name);
        intake2 = hMap.get(DcMotorEx.class, name2);
    }

    public void init() {
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentIntake1State = IntakeStates.INTAKE_OFF;
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setPower(0);
        intake1.setVelocity(0);

        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentIntake2State = IntakeStates.INTAKE_OFF;
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setPower(0);
        intake2.setVelocity(0);
    }

    @Override
    public void periodic(){
       IntakeStates.INTAKE_ON.SetStatePower(intakeParameters.INTAKE_ON_POWER);
       IntakeStates.INTAKE_SLOW.SetStatePower(intakeParameters.INTAKE_SLOW_POWER);
       IntakeStates.INTAKE_REVERSE.SetStatePower(intakeParameters.INTAKE_REVERSE_POWER);

       //Add the Winch Motor State to our loop telemetry packet
       MatchConfig.telemetryPacket.put("Current Intake 1 State", currentIntake1State);
       MatchConfig.telemetryPacket.put("Current Intake 2 State", currentIntake2State);
    }
}
