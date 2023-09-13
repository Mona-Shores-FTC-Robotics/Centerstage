package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name = "Test Chassis", group =  "Chassis Bot")

public class TestChassis extends LinearOpMode {

    Robot robot = Robot.createInstance(this);

    //Gyro gyro = Robot.getInstance().getGyro();
    //DriveTrain MecDrive = Robot.getInstance().getDriveTrain();

    private DriveTrain MecDrive;

    //    GamepadHandling GamePads = new GamepadHandling(this);
    private final ElapsedTime runtime = new ElapsedTime();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        MecDrive = Robot.getInstance().getDriveTrain();
        boolean driveMethodSpeedControl = false; // false = power control, true = speed control

        boolean manualControl = false;
        boolean autoControl = false;

        double testRunStart = -2;
        double maximumSpeed[] = {0, 0, 0, 0};
        double timeTo200RPM[] = {0, 0, 0, 0};
        double distanceTraveled[] = {0, 0, 0, 0};
        String maxSpeed = null;
        String timeTo200 = null;
        String distTraveled = null;
        double autoDriveInput = 1;
        double P = 10;
        double I = 3;
        double D = 0;
        double F = 12;
        double F_INCREMENT = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
        }

        runtime.reset();
        while (opModeIsActive()) {
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);


            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                driveMethodSpeedControl = !driveMethodSpeedControl;
            }


            MecDrive.drive = - currentGamepad1.left_stick_y;
            MecDrive.strafe = currentGamepad1.left_stick_x;
            MecDrive.turn = currentGamepad1.right_stick_x;

            if (Math.abs(currentGamepad1.left_stick_x) + Math.abs(currentGamepad1.left_stick_y)+Math.abs(currentGamepad1.right_stick_x) > .1){
                manualControl = true;
                MecDrive.drive = - currentGamepad1.left_stick_y;
                MecDrive.strafe = currentGamepad1.left_stick_x;
                MecDrive.turn = currentGamepad1.right_stick_x;
            }
            else {
                manualControl = false;
            }

            telemetry.addData("Speed Control", driveMethodSpeedControl);
            telemetry.addData("Power Control", !driveMethodSpeedControl);

            if(driveMethodSpeedControl && manualControl){

                MecDrive.mecanumDriveSpeedControl();
            }
            else if(manualControl) {
                telemetry.addData("Drive Method", "Power Control");
                MecDrive.mecanumDrivePowerControl();
            }
            else {
                MecDrive.drive = MecDrive.strafe = MecDrive.turn = 0;
                MecDrive.mecanumDrivePowerControl();
            }

            telemetry.addData("ticks", MecDrive.driveMotor[0].getCurrentPosition());
            telemetry.addData("PIDF Coefficients", MecDrive.driveMotor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("input power", autoDriveInput);
            telemetry.addData("maximum speed", maxSpeed);
            telemetry.addData("time to reach 200 rpm", timeTo200);
            telemetry.addData("Wheel Rotations", distTraveled);
            telemetry.update();
        }
    }
}
