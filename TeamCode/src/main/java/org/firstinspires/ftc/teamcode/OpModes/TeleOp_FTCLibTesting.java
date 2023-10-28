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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;

@TeleOp(name="TeleOp_FTCLibTesting")
public class TeleOp_FTCLibTesting extends LinearOpMode
{
    public Robot robot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override public void runOpMode()
    {
        /** Create and Initialize the robot **/
        robot = Robot.createInstance(this, Robot.RobotType.ROBOT_SCORING_ARM, Robot.OpModeType.TELEOP);

        /** Initialize Gamepad and Robot - Order Important **/
        GamepadHandling.init();
        robot.init();

        /** Setup Telemetry for Driver Station and FTCDashboard **/
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /** Setup Button Bindings **/
        GamepadHandling.bindDriverGamepadButtons(robot.opModeType, robot.robotType);
        GamepadHandling.bindOperatorGamepadButtons(robot.opModeType, robot.robotType);

        telemetry.clearAll();

        while (opModeInInit()) {
            telemetry.update();
            sleep(10);
        }

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();

            //Read all buttons
            GamepadHandling.getDriverGamepad().readButtons();

            telemetry.update();
        }
        robot.getVisionSubsystem().getVisionPortal().close();
    }


}
