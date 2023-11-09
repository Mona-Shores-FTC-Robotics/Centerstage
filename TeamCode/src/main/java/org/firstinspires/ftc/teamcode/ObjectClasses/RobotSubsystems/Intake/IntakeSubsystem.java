package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public static class IntakeParameters {
        public double INTAKE_OFF_POWER = 0;
        public double INTAKE_REVERSE_VELOCITY = -30;
        public double INTAKE_ON_VELOCITY = 30;
        public double INTAKE_ON_POWER = .8;
    }
    public static IntakeParameters intakeParameters = new IntakeParameters();

    public enum IntakeStates {
        INTAKE_ON (30),
        INTAKE_REVERSE (-150),
        INTAKE_OFF (0);

        public double velocity;

        IntakeStates(double vel) {
            this.velocity = vel;
        }
        void SetStateVelocity(double vel){
            this.velocity = vel;
        }
    }

    public DcMotorEx intake;
    public PIDFCoefficients pidfCoefficients;

    public IntakeStates currentState;
    public double power;

    public IntakeSubsystem(final HardwareMap hMap, final String name){
        intake = hMap.get(DcMotorEx.class, name);
    }

    public void init() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(intakeParameters.INTAKE_OFF_POWER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentState = IntakeStates.INTAKE_OFF;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        power = intakeParameters.INTAKE_OFF_POWER;
        intake.setPower(0);
        intake.setVelocity(0);
    }

    @Override
    public void periodic(){
        //this is the only way i can find to make it tunable
       IntakeStates.INTAKE_ON.SetStateVelocity(intakeParameters.INTAKE_ON_VELOCITY);

       //todo code for turning on lights when we sense a pixel in the intake should go here
    }
}
