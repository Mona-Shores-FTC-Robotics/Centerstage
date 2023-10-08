package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class Arm {
    public Servo arm;
    public LinearOpMode activeOpMode;

    public enum ArmPositions {
        LEFT_OUTTAKE (1.0, 0.0),
        INTAKE (0.67, 0.4),
        RIGHT_OUTTAKE (0.33, 0.8),
        FRONT_OUTTAKE (0.0, 1.2);
        private double value;
        private double timeToLeftOuttake;

        private ArmPositions(double value, double timeToLeftOuttake) {
            this.value = value;
            this.timeToLeftOuttake = timeToLeftOuttake;
        }
    }
    public ArmPositions armTargetPosition = ArmPositions.INTAKE;
    public ArmPositions armPosition = ArmPositions.INTAKE;
    public ElapsedTime armRotateTimer = new ElapsedTime();

    boolean safeToRotate = false;
    boolean rotating = false;

    public Arm() {

    }
    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        arm = Robot.getInstance().getHardwareMap().servo.get("turret_servo");
        //set arm at intake position
        armTargetPosition = armPosition = ArmPositions.INTAKE;
        arm.setPosition(armTargetPosition.value);
    }

    public void rotateArm(ArmPositions targetPosition){

        armTargetPosition = targetPosition;

        if (safeToRotate && !rotating && armPosition != armTargetPosition) {
            arm.setPosition(armTargetPosition.value);
            armRotateTimer.reset();
            rotating = true;
        }
        else if(rotating && Math.abs(armTargetPosition.timeToLeftOuttake-armPosition.timeToLeftOuttake) < armRotateTimer.seconds()){
            armPosition = armTargetPosition;
            rotating = false;
        }
    }
}
