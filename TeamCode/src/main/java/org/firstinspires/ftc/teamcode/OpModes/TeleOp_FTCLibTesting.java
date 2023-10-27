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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name="TeleOp_FTCLib")
public class TeleOp_FTCLibTesting extends LinearOpMode
{

    /** Create the robot **/
    Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_SCORING_ARM);

    @Override public void runOpMode()
    {
        //Initialize the Robot
        robot.initialize();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize the Gamepads
        GamepadHandling.init();

        telemetry.clearAll();

        while (opModeInInit()) {
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            telemetry.update();
            sleep(10);
        }

        //Start the TeleOp Timer
        robot.getTeleOpRuntime().reset();

        while (opModeIsActive())
        {

            //Store the previous loop's gamepad values and new current gamepad values
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeCurrentGamepadValues();

            //This is a loop that we can use for tuning

            if (GamepadHandling.driverButtonPressed("x")) {
//                runBlocking(Robot.getInstance().getScoringArm().grabAndScorePixelOnBackdropLow);
                Robot.getInstance().getLiftSlideSubsystem().liftToLowHeight().run(new TelemetryPacket());
            }

            if (GamepadHandling.driverButtonPressed("y")) {

                Robot.getInstance().getLiftSlideSubsystem().liftToHighHeight().run(new TelemetryPacket());
//                runBlocking(Robot.getInstance().getScoringArm().grabAndScorePixelOnBackdropMid);
            }

            if (GamepadHandling.driverButtonPressed("b")) {
//                runBlocking(Robot.getInstance().getScoringArm().grabAndScorePixelOnBackdropHigh);
                runBlocking(Robot.getInstance().getScoringArm().makeGrabAndScorePixelOnBackdropMid());
            }

            if (GamepadHandling.driverButtonPressed("a")) {
//                runBlocking(Robot.getInstance().getScoringArm().grabAndScorePixelOnBackdropHigh);
                runBlocking(new ParallelAction(
                        Robot.getInstance().getDriveSubsystem().actionBuilder(Robot.getInstance().getDriveSubsystem().pose)
                                .strafeTo(new Vector2d(0,5))
                                .build(),
                        Robot.getInstance().getScoringArm().makeGrabAndScorePixelOnBackdropMid()));

            }
            telemetry.update();
        }
        robot.getVisionSubsystem().getVisionPortal().close();
    }


}
