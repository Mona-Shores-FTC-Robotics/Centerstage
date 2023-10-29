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
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.defaultCommand;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo0;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo180;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo270;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo90;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.Actions.CenterstageActions;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands.MoveToPoint;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.IntakeTestingDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.VisionDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

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

        //Setup Telemetry for Driver Station and FTCDashboard
        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /* Setup Button Bindings **/
        new VisionDriverBindings(GamepadHandling.getDriverGamepad());

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            //todo does this print the final side/color from auto?
            telemetry.addData("Alliance Color", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal());
            telemetry.addData("Side of the Field", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getSideOfFieldFinal());

            telemetry.update();
            sleep(10);
        }

        robot.getVisionSubsystem().telemetryForInitProcessing();
        telemetry.update();

        //After Init switch the vision processing to AprilTags
        robot.getVisionSubsystem().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        //set the Default command

        CommandScheduler.getInstance().setDefaultCommand(robot.getDriveSubsystem(), defaultCommand);

        Command testCommand = new MoveToPoint(Robot.getInstance().getDriveSubsystem(), 20, 0);

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();
            GamepadHandling.getDriverGamepad().readButtons();

            //Update Gyro values
            robot.getGyroSubsystem().UpdateGyro();

            //Look for AprilTags
            robot.getVisionSubsystem().LookForAprilTags();

            Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

            if (GamepadHandling.getDriverGamepad().wasJustPressed(GamepadKeys.Button.A)) {
                telemetry.addData("pose", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);
                testCommand.schedule();
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

        robot.getVisionSubsystem().getVisionPortal().close();
        CommandScheduler.getInstance().reset();
    }
}

