package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class PixelIntake {
    public static double motorFwd;
    public static double motorRev;

    public  DcMotor testIntakeMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    LinearOpMode activeOpMode = null;

//I like turtles

    public PixelIntake(){

    }

    public void init() {

        activeOpMode = Robot.getInstance().getActiveOpMode();
        // Save reference to Hardware map
        hwMap = Robot.getInstance().getHardwareMap();

        testIntakeMotor = hwMap.get(DcMotor.class, "test"); //Change this name to the motor name in the config
        testIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        testIntakeMotor.setPower(0);
    }

    public void move() {
        //Test Mechanism
        motorFwd = GamepadHandling.getCurrentOperatorGamepad().right_trigger;
        motorRev = -GamepadHandling.getCurrentOperatorGamepad().left_trigger;

        if(motorFwd > 0){
            testIntakeMotor.setPower(motorFwd);
        }
        else if (motorRev < 0) {
            testIntakeMotor.setPower(motorRev);
        }else testIntakeMotor.setPower(0);

    }
}
