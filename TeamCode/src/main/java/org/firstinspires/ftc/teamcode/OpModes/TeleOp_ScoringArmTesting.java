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

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands.defaultDriveCommand;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotCommands.defaultLiftSlideCommand;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.ScoringArmTestingDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Utility.TelemetryMona;

@TeleOp(name="TeleOp_ScoringArmTesting")
public class TeleOp_ScoringArmTesting extends LinearOpMode
{
    public Robot robot;

    @Override public void runOpMode()
    {
        /* Create and Initialize the robot **/
        Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_SCORING_ARM, Robot.OpModeType.TELEOP);

        /* Initialize Gamepad and Robot - Order Important **/
        GamepadHandling.init();
        robot.init();

        telemetry.clearAll();

        /* Setup Button Bindings **/
        new ScoringArmTestingDriverBindings(GamepadHandling.getDriverGamepad());

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            //todo does this print the final side/color from auto?
//            telemetry.addData("Alliance Color", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getAllianceColorFinal());
//            telemetry.addData("Side of the Field", Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().getSideOfFieldFinal());

            TelemetryMona.scoringArmTestingButtons();
            telemetry.update();
            sleep(10);
        }

        //Start the TeleOp Timer
        robot.getTeleOpTimer().reset();

        //Set the default lift command
        CommandScheduler.getInstance().setDefaultCommand(robot.getLiftSlideSubsystem(), defaultLiftSlideCommand);

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();

            //Read all buttons
            GamepadHandling.getDriverGamepad().readButtons();

            //Right Trigger shows some telemetry about the buttons
            if (ScoringArmTestingDriverBindings.rightTrigger.isDown()) {
                TelemetryMona.scoringArmTestingButtons();
            }

            telemetry.addData("Ticks", Robot.getInstance().getLiftSlideSubsystem().liftSlide.getCurrentPosition());

            sleep(10);
            telemetry.update();
        }
        robot.getVisionSubsystem().getVisionPortal().close();
    }
}

