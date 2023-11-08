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

package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.VisionDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{
    @Override public void runOpMode()
    {
        //Create the Robot
        Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);

        //Initialize the Game-pads
        GamepadHandling.init();

        //Initialize the Robot
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        /* Setup Button Bindings **/
        new VisionDriverBindings(GamepadHandling.getDriverGamepad());

        while (opModeInInit()) {
            VisionTelemetry.telemetryForInitProcessing();
            GamepadHandling.getDriverGamepad().readButtons();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        //Set the starting pose of the robot
        Robot.getInstance().getVisionSubsystem().setStartingPose(MatchConfig.finalAllianceColor, MatchConfig.finalSideOfField);

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().resetAbsoluteYaw();

        //Start the TeleOp Timer
        ElapsedTime teleOpTimer = new ElapsedTime();
        teleOpTimer.reset();

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();
            GamepadHandling.getDriverGamepad().readButtons();

            //Look for AprilTags
            Robot.getInstance().getVisionSubsystem().LookForAprilTags();

            //Add AprilTag Telemetry
            if (gamepad1.left_trigger>.1) {
                telemetry.addData("Alliance Color", MatchConfig.finalAllianceColor);
//                Robot.getInstance().getVisionSubsystem().telemetryAprilTag();
                Robot.getInstance().getGyroSubsystem().telemetryGyro();
            }

            //Add DriveTrain Telemetry
            if (gamepad1.right_trigger>.1) {
                Robot.getInstance().getDriveSubsystem().mecanumDrive.telemetryDriveTrain();
                Robot.getInstance().getGyroSubsystem().telemetryGyro();
                telemetry.addData("leftstick y", GamepadHandling.getDriverGamepad().getLeftY());
                telemetry.addData("leftstick x", GamepadHandling.getDriverGamepad().getLeftX() );
                telemetry.addData("rightstick x", GamepadHandling.getDriverGamepad().getRightX());
            }

            telemetry.update();
        }
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getDriveSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getGyroSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getVisionSubsystem());
        Robot.reset();
    }
}

