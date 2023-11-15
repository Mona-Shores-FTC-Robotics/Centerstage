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
        public double INTAKE_SLOW = .2;
    }
    public static IntakeParameters intakeParameters = new IntakeParameters();

    public enum IntakeStates {
        INTAKE_ON (30, 1),
        INTAKE_SLOWER (30, .2),
        INTAKE_REVERSE (-150,-.8),
        INTAKE_OFF (0, 0);

        public double velocity;
        public double power;

        IntakeStates(double vel, double pow) {
            this.velocity = vel; this.power = pow;
        }
        void SetStateVelocity(double vel){
            this.velocity = vel;
        }
    }
    //SECOND INTAKE MOTOR - PORT 3 EXPANSION HUB
    public DcMotorEx intake;
    public DcMotorEx intake2;

    public PIDFCoefficients pidfCoefficients;

    public IntakeStates currentState;
    public double power;

    public IntakeSubsystem(final HardwareMap hMap, final String name, final String name2){
        intake = hMap.get(DcMotorEx.class, name);
        intake2 = hMap.get(DcMotorEx.class, name2);
    }

    public void init() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentState = IntakeStates.INTAKE_OFF;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        power = intakeParameters.INTAKE_OFF_POWER;
        intake.setPower(0);
        intake.setVelocity(0);

        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentState = IntakeStates.INTAKE_OFF;
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        power = intakeParameters.INTAKE_OFF_POWER;
        intake2.setPower(0);
        intake2.setVelocity(0);
    }

    @Override
    public void periodic(){
        //this is the only way i can find to make it tunable
       IntakeStates.INTAKE_ON.SetStateVelocity(intakeParameters.INTAKE_ON_VELOCITY);
       IntakeStates.INTAKE_SLOWER.SetStateVelocity(intakeParameters.INTAKE_SLOW);

       //todo code for turning on lights when we sense a pixel in the intake should go here
    }
}
