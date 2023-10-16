package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class Shoulder {

    private int SHOULDER_TIMEOUT_SECONDS = 3;
    public Servo shoulder;
    public LinearOpMode activeOpMode;

    public enum ShoulderPositions {
        INTAKE (1.0),
        BACKDROP (0),
        UNKNOWN (0);

        private double value;

        ShoulderPositions(double value) {
            this.value = value;
        }

        void SetPosition(double v) {
            this.value = v;
        }

        double GetPosition() {
            return this.value;
        }
    }
    public ShoulderPositions shoulderTargetPosition = ShoulderPositions.INTAKE;
    public ShoulderPositions shoulderPosition = ShoulderPositions.INTAKE;
    public ShoulderPositions unknownShoulderPosition = ShoulderPositions.UNKNOWN;
    public ElapsedTime shoulderRotateTimer = new ElapsedTime();

    private boolean safeToRotate = false;
    private boolean rotating = false;

    public Shoulder() {

    }

    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        shoulder = Robot.getInstance().getHardwareMap().servo.get("shoulder");

        //set shoulder at intake position
        shoulderTargetPosition = shoulderPosition = ShoulderPositions.INTAKE;
        shoulder.setPosition(shoulderTargetPosition.value);
    }

    public void Rotate(ShoulderPositions targetPosition){
        shoulderTargetPosition = targetPosition;
        if (safeToRotate && !rotating && shoulderPosition != shoulderTargetPosition) {
            shoulder.setPosition(shoulderTargetPosition.value);
            shoulderRotateTimer.reset();
            rotating = true;
        }
        else if(rotating && shoulderRotateTimer.seconds() < SHOULDER_TIMEOUT_SECONDS){
            shoulderPosition = shoulderTargetPosition;
            rotating = false;
        } else
        {
            Robot.getInstance().getActiveOpMode().telemetry.addLine("Shoulder timeout exceeded");
            shoulderPosition = unknownShoulderPosition;
            shoulderPosition.SetPosition(shoulder.getPosition());
            rotating = false;
        }
    }

    public boolean isRotating() {
        return rotating;
    }

    public void setRotating(boolean rotating) {
        this.rotating = rotating;
    }

    public boolean isSafeToRotate() {
        return safeToRotate;
    }

    public void setSafeToRotate(boolean safeToRotate) {
        this.safeToRotate = safeToRotate;
    }

}
