package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name = "Chassis Only Teleop", group = "Basic Chassis")
public class BasicChassis extends LinearOpMode{

    Robot robot = Robot.createInstance(this);
    ElapsedTime runtime = robot.getRuntime();
    Gyro gyro = robot.getGyro();
    DriveTrain drivetrain = robot.getDriveTrain();



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

        runtime.reset();
        while (opModeIsActive()){
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            // Read sensors and perform sensor calculations
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                gyro.UpdateGyro(runtime);
            }

            // Rumble Control
            // Replace comment with call to rumble control method

            // Driver Controls
            drivetrain.driveModeSelection(currentGamepad1,gyro.turnAngle,gyro.tiltAngle,gyro.tiltVelocity,gyro.tiltAccel);

            // Operator Controls
            // Replace this comment with methods for operator controls
        }
    }
}
