package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name = "Teleop", group = "Robot")
public class TELEOP_Robot extends LinearOpMode{
    Robot robot = Robot.createInstance(this);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

        }

        robot.getRuntime().reset();
        while (opModeIsActive()){
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            // Read sensors and perform sensor calculations
            // Rumble Control
            // Replace comment with call to rumble control method
            // Calculations & Logic
            // Driver and Operator controls execution

        }
    }
}
