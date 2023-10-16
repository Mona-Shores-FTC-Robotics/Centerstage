package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class EndEffector {
    public Servo endEffector;

    public LinearOpMode activeOpMode;
    private double SECONDS_TO_OUTTAKE = 0.5;

    public enum EndEffectorStates {
        GRAB_TWO_PIXELS (1),
        RELEASE_BOTH_PIXELS (0),
        READY_FOR_PIXELS(0);
        private double endEffectorPosition;

        private EndEffectorStates(double endEffectorPosition) {
            this.endEffectorPosition = endEffectorPosition;
        }
    }
    public EndEffectorStates currentIntakeState;
    public EndEffector() {

    }

    public void init() {
        activeOpMode = Robot.getInstance().getActiveOpMode();
        HardwareMap ahwMap = Robot.getInstance().getHardwareMap();
        endEffector = ahwMap.servo.get("endeffector");
        currentIntakeState = EndEffectorStates.READY_FOR_PIXELS;
        endEffector.setPosition(currentIntakeState.endEffectorPosition);

    }

    public void GrabTwoPixels() {
        endEffector.setPosition(EndEffectorStates.GRAB_TWO_PIXELS.endEffectorPosition);
    }

    public void ReleaseBothPixels() {
        endEffector.setPosition(EndEffectorStates.RELEASE_BOTH_PIXELS.endEffectorPosition);
    }

}
