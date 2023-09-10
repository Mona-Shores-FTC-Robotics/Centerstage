package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeOuttake {
    public Servo pivot;
    public Servo leftIntake;
    public Servo rightIntake;
    public LinearOpMode activeOpMode;
    private double SECONDS_TO_OUTTAKE = 0.5;

    public enum IntakeStates {
        INTAKE (0.67, 1.0, 0.0),
        MANUAL_INTAKE (.67, 1.0, 0.0),
        HOLD_CONE (1.0, .5, .5),
        OUTTAKE (0.0, .4, .6),
        READY_TO_INTAKE (0.67, .5, .5);
        private double pivotServo;
        private double leftIntakeServo;
        private double rightIntakeServo;

        private IntakeStates(double pivotServo, double leftIntakeServo, double rightIntakeServo) {
            this.pivotServo = pivotServo;
            this.leftIntakeServo = leftIntakeServo;
            this.rightIntakeServo = rightIntakeServo;
        }
    }
    public IntakeStates currentIntakeState;
    public IntakeStates lastIntakeState;
    boolean outtakeComplete = true;

    public ElapsedTime outtakeTimer = new ElapsedTime();

    boolean safeToRotate = false;
    boolean rotating = false;

    public IntakeOuttake() {
        activeOpMode = Robot.getInstance();
    }

    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        activeOpMode = opMode;
        pivot = ahwMap.servo.get("pivot");
        leftIntake = ahwMap.servo.get("Left Intake");
        rightIntake = ahwMap.servo.get("Right Intake");

        lastIntakeState = currentIntakeState = IntakeStates.HOLD_CONE;

        pivot.setPosition(currentIntakeState.pivotServo);
        leftIntake.setPosition(currentIntakeState.leftIntakeServo);
        rightIntake.setPosition(currentIntakeState.rightIntakeServo);
    }
    public void teleopInit(HardwareMap ahwMap, LinearOpMode opMode) {
        activeOpMode = opMode;
        pivot = ahwMap.servo.get("pivot");
        leftIntake = ahwMap.servo.get("Left Intake");
        rightIntake = ahwMap.servo.get("Right Intake");

        lastIntakeState = currentIntakeState = IntakeStates.INTAKE;

        pivot.setPosition(currentIntakeState.pivotServo);
        leftIntake.setPosition(currentIntakeState.leftIntakeServo);
        rightIntake.setPosition(currentIntakeState.rightIntakeServo);
    }

    public void runIntakeOuttake (IntakeStates targetState){


        if(targetState == IntakeStates.OUTTAKE && currentIntakeState != IntakeStates.OUTTAKE){
            outtakeTimer.reset();
            outtakeComplete = false;
            currentIntakeState = IntakeStates.OUTTAKE;
        } else if (!outtakeComplete && outtakeTimer.seconds() > SECONDS_TO_OUTTAKE) {
            currentIntakeState = IntakeStates.READY_TO_INTAKE;
            outtakeComplete = true;
        } else if (targetState != IntakeStates.INTAKE && currentIntakeState == IntakeStates.INTAKE) {
            currentIntakeState = IntakeStates.HOLD_CONE;
        } else { currentIntakeState = targetState;
        }

        pivot.setPosition(currentIntakeState.pivotServo);
        leftIntake.setPosition(currentIntakeState.leftIntakeServo);
        rightIntake.setPosition(currentIntakeState.rightIntakeServo);
    }
}
