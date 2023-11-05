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

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.END_GAME_TIME;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.CenterstageDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.CenterstageOperatorBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;

@TeleOp(name="TeleOp_CenterStage")
public class TeleOp_CenterStage extends LinearOpMode
{
    private ElapsedTime teleOpTimer;

    @Override
    public void runOpMode()
    {
        /* Create the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_CENTERSTAGE);

        /* Initialize Gamepad and Robot - Order Important **/
        GamepadHandling.init();
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        /* Setup Button Bindings **/
        new CenterstageDriverBindings(GamepadHandling.getDriverGamepad());
        new CenterstageOperatorBindings(GamepadHandling.getOperatorGamepad());

        telemetry.clearAll();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            //todo does this print the final side/color from auto?
            telemetry.addData("Alliance Color", MatchConfig.finalAllianceColor);

            telemetry.update();
            sleep(10);
        }

        //Start the TeleOp Timer
        teleOpTimer = new ElapsedTime();
        teleOpTimer.reset();

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();

            //Read all buttons
            GamepadHandling.getDriverGamepad().readButtons();

            EndGameRumble();
            ActivateEndGameButtons();

            telemetry.update();
            sleep(10);
        }
        Robot.getInstance().getVisionSubsystem().getVisionPortal().close();
    }

    private void EndGameRumble() {
        if ( teleOpTimer.seconds()>END_GAME_TIME-5){
            //Rumble the controllers
            //Flash lights on controller?
            //Flash lights on robot?
        }
    }

    void ActivateEndGameButtons(){
        if ( teleOpTimer.seconds()>END_GAME_TIME){
            //check buttons for Wench and drone
            //Right Trigger shows some telemetry about the buttons
            if (CenterstageOperatorBindings.rightTrigger.isDown()) {
                //schedule the Wench release command here
            }
            if (CenterstageOperatorBindings.leftTrigger.isDown()) {
                //schedule the Drone release command here
            }
        }
    }

}
