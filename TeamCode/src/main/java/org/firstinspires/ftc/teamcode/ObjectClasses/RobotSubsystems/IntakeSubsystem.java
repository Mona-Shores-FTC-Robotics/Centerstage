package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;

public class IntakeSubsystem extends SubsystemBase {
    public static double motorFwd;
    public static double motorRev;

    public DcMotor intakeMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    LinearOpMode activeOpMode = null;

    public IntakeSubsystem(final HardwareMap hMap, final String name){
        intakeMotor = hMap.get(DcMotor.class, name);
    }

    public void init() {
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0);
    }

    public void move() {
        //Test Mechanism
        motorFwd = GamepadHandling.getDriverGamepad().getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        motorRev = -GamepadHandling.getDriverGamepad().getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);;

        if(motorFwd > 0){
            intakeMotor.setPower(motorFwd);
        }
        else if (motorRev < 0) {
            intakeMotor.setPower(motorRev);
        }else intakeMotor.setPower(0);
    }

    @Override
    public void periodic(){
        //This will be called once per scheduler run
    }
}
