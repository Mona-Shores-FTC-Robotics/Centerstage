/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;


@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{

    /** Create the robot **/
    Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);
    DriveSubsystem driveSubsystem;

    @Override public void runOpMode()
    {
        //Initialize the Robot
        robot.initialize();

        driveSubsystem = robot.getDriveSubsystem();

        //initialize the Gamepads
        GamepadHandling.init();
        robot.getVisionSubsystem().SwitchToInitVisionProcessor();

        //Create the action for drawing the Robot during TeleOp
        DriveSubsystem.DrawCurrentPosition drawTeleOpRobot = driveSubsystem.new DrawCurrentPosition();
        DriveSubsystem.SendSpeedAndPositionDataToDashboard sendSpeedAndPositionDataToDashboard = driveSubsystem.new SendSpeedAndPositionDataToDashboard();

        while (opModeInInit()) {
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            // Add Vision Init Processor Telemetry
            robot.getVisionSubsystem().telemetryForInitProcessing();

            GamepadHandling.lockColorAndSide();

            telemetry.update();

            sleep(10);
        }

        robot.getVisionSubsystem().telemetryForInitProcessing();
        telemetry.update();

        //After Init switch the vision processing to AprilTags
        robot.getVisionSubsystem().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        while (opModeIsActive())
        {

            //Store the previous loop's gamepad values and new current gamepad values
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            //Update Gyro values
            robot.getGyroSubsystem().UpdateGyro();

            //Look for AprilTags
            robot.getVisionSubsystem().LookForAprilTags();

            //Process the Driver Controls
            GamepadHandling.DriverControls();

            //Process the Operator Controls
            GamepadHandling.OperatorControls();

            //Drive the Robot (manual if driver controls are active - or automatically if flag set)
            robot.getDriveController().setDriveStrafeTurnValues();

            driveSubsystem.mecanumDriveSpeedControl();

            driveSubsystem.updatePoseEstimate();

            //Add AprilTag Telemetry
            if (gamepad1.left_trigger>.1) {

                telemetry.addData("Alliance Color", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal());
                telemetry.addData("Side of the Field", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getSideOfFieldFinal());
                telemetry.addData("Team Prop Location", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocationFinal());

                robot.getVisionSubsystem().telemetryAprilTag();
            }

            //Add DriveTrain Telemetry
            if (gamepad1.right_trigger>.1) {
                robot.getDriveSubsystem().telemetryDriveTrain();
                robot.getGyroSubsystem().telemetryGyro();

                telemetry.addData("leftstick y", GamepadHandling.getCurrentDriverGamepad().left_stick_y );
                telemetry.addData("leftstick x", GamepadHandling.getCurrentDriverGamepad().left_stick_x );
                telemetry.addData("rightstick x", GamepadHandling.getCurrentDriverGamepad().right_stick_x );

            }
            //this sends speed and position data to the dashboard
            runBlocking(sendSpeedAndPositionDataToDashboard);

            //this sends info to dashboard to draw our robot on the field
            runBlocking(drawTeleOpRobot);

            telemetry.update();
        }
        robot.getVisionSubsystem().getVisionPortal().close();
    }
}


//This is how RR runs its drive
//            mecanumDrive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));