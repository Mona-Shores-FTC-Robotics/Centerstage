package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Vision;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TeleOp_Test_OpenCVDetection")

public class TeleOp_Test_OpenCVDetection extends LinearOpMode {

    Robot robot = Robot.createInstance(this);

    private DriveTrain MecDrive;
    private Vision vision = new Vision();
    private int finalTeamPropVision;

    //    GamepadHandling GamePads = new GamepadHandling(this);
    private final ElapsedTime runtime = new ElapsedTime();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        //This OpMode uses the robot with a Chassis, Camera, and Gyro
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);
        robot.initialize(hardwareMap);

        MecDrive = Robot.getInstance().getDriveTrain();
        boolean driveMethodSpeedControl = false; // false = power control, true = speed control
        boolean manualControl = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

        }
        telemetry.addData("Final Team Element Location", finalTeamPropVision);
        telemetry.update();

        runtime.reset();

        while (opModeIsActive()) {
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            /** Driver Controls**/
            MecDrive.drive();

            //Telemetry
            telemetry.addData("Speed Control", driveMethodSpeedControl);
            telemetry.addData("Power Control", !driveMethodSpeedControl);
            telemetry.addData("ticks", MecDrive.driveMotor[0].getCurrentPosition());
            telemetry.addData("PIDF Coefficients", MecDrive.driveMotor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }

    }
}
