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

@TeleOp(name = "Full Teleop", group = "Robot")
public class TELEOP_Robot extends LinearOpMode{

    //DriveTrain MecDrive = new DriveTrain(this);
    private final ElapsedTime runtime = new ElapsedTime();
    Robot robot = new Robot(this, runtime);
    //Gyro Gyro = new Gyro(this);

//    GamepadHandling GamePads = new GamepadHandling(this);



    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {

        //These should be unnecessary...
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //MecDrive.init(hardwareMap,this);
        robot.init(hardwareMap);
        //Gyro.init(hardwareMap);

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
            robot.readSensors();

            // Rumble Control
            // Replace comment with call to rumble control method

            // Calculations & Logic
            robot.operatorGamepadTranslation(currentGamepad2, previousGamepad2);
            robot.mechanismLogic(currentGamepad2);

            // Driver and Operator controls execution
            robot.executeFunctions(currentGamepad1, currentGamepad2);
        }

    }
}
