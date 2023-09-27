package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling.currentOperatorGamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestIntake {
    public static double motorFwd;
    public static double motorRev;

    public  DcMotor TestIntake = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
//I like turtles

    public TestIntake(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        TestIntake = hwMap.get(DcMotor.class, "TestIntake");
        TestIntake.setDirection(DcMotor.Direction.FORWARD);
        TestIntake.setPower(0);
    }

    public void testIntake() {
        //Test Mechanism
        motorFwd = GamepadHandling.currentOperatorGamepad.right_trigger;
        motorRev = -GamepadHandling.currentOperatorGamepad.left_trigger;

        if(motorFwd > 0){
            TestIntake.setPower(motorFwd);
        }
        else if (motorRev < 0) {
            TestIntake.setPower(motorRev);
        }else

        TestIntake.setPower(0);



    }
}
