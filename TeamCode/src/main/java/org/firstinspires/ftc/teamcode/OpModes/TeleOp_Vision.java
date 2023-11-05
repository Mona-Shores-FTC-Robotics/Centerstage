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

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.VisionDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;

@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{

    @Override public void runOpMode()
    {
        //Create the Robot
        Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_VISION, Robot.OpModeType.TELEOP);

        //Initialize the Game-pads
        GamepadHandling.init();

        //Initialize the Robot
        robot.init();

        /* Setup Button Bindings **/
        new VisionDriverBindings(GamepadHandling.getDriverGamepad());

        InitVisionProcessor.AllianceColor allianceColor = Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry

            //todo does this print the final side/color from auto?
            telemetry.addData("Alliance Color", allianceColor);

            telemetry.update();
            sleep(10);
        }

        robot.getVisionSubsystem().telemetryForInitProcessing();
        telemetry.update();

        //Switch the vision processing to AprilTags
        robot.getVisionSubsystem().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpTimer().reset();



        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();
            GamepadHandling.getDriverGamepad().readButtons();

            //Update Gyro values
            robot.getGyroSubsystem().UpdateGyro();

            //Look for AprilTags
            robot.getVisionSubsystem().LookForAprilTags();

            //Update robot pose
            Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

            //drive backwards and rotate toward wing
            //we could make this cleaner, but for now I like knowing this is based on Alliance Color - easy to forget if we bury it
            if (GamepadHandling.getDriverGamepad().wasJustPressed(GamepadKeys.Button.Y))
            {
                if (Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal() == InitVisionProcessor.AllianceColor.RED) {
                    RobotCommands.redBackdropBackup.schedule();
                } else{
                    RobotCommands.blueBackdropBackup.schedule();
                }
            }

            //Add AprilTag Telemetry
            if (gamepad1.left_trigger>.1) {

                telemetry.addData("Alliance Color", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal());
                telemetry.addData("Side of the Field", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getSideOfFieldFinal());
                telemetry.addData("Team Prop Location", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getTeamPropLocationFinal());

                robot.getVisionSubsystem().telemetryAprilTag();
            }

            //Add DriveTrain Telemetry
            if (gamepad1.right_trigger>.1) {
                robot.getDriveSubsystem().mecanumDrive.telemetryDriveTrain();
                robot.getGyroSubsystem().telemetryGyro();

                telemetry.addData("leftstick y", GamepadHandling.getDriverGamepad().getLeftY());
                telemetry.addData("leftstick x", GamepadHandling.getDriverGamepad().getLeftX() );
                telemetry.addData("rightstick x", GamepadHandling.getDriverGamepad().getRightX());
            }

            telemetry.update();
        }
        CommandScheduler.getInstance().cancelAll();
        robot.getVisionSubsystem().getVisionPortal().close();
    }
}

