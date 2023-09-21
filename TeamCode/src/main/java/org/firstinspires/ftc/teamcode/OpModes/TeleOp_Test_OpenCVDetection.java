package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        robot.initialize(hardwareMap);
        vision.init(hardwareMap);

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
            //Left Bumper switches between Speed Control and Power Control
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                driveMethodSpeedControl = !driveMethodSpeedControl;
            }

            //Checks whether the Driver sticks have moved
            if (GamepadHandling.gamepadIsActive(currentGamepad1))
            {
                manualControl = true;
                MecDrive.drive = - currentGamepad1.left_stick_y;
                MecDrive.strafe = currentGamepad1.left_stick_x;
                MecDrive.turn = currentGamepad1.right_stick_x;
            }
            else {
                manualControl = false;
            }


            //Call the approriate method to drive the robot
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


            //Telemetry
            telemetry.addData("Speed Control", driveMethodSpeedControl);
            telemetry.addData("Power Control", !driveMethodSpeedControl);
            telemetry.addData("ticks", MecDrive.driveMotor[0].getCurrentPosition());
            telemetry.addData("PIDF Coefficients", MecDrive.driveMotor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }

    }
}
