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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{
    /** Create the robot **/
    Robot robot = Robot.createInstance(this);

    @Override public void runOpMode()
    {
        //Set the type of Robot
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);

        //Initialize the Robot
        robot.initialize(Robot.getInstance().getHardwareMap());

        //initialize the Gamepads
        GamepadHandling.init();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            Robot.getInstance().getVision().getInitVisionProcessor().telemetryForInitProcessing();

            //TODO: write code to allow user to override alliance color and sideOfField determined by vision

            telemetry.update();
        }

        //Display the initVision telemetry a final time
        Robot.getInstance().getVision().getInitVisionProcessor().telemetryForInitProcessing();
        telemetry.update();

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVision().SwitchToAprilTagProcessor();

        //Start the TeleOp Timer
        Robot.getInstance().getTeleOpRuntime().reset();

        while (opModeIsActive())
        {
            //Store the previous loop's gamepad values and new current gamepad values
            GamepadHandling.storeGamepadValuesFromLastLoop();
            GamepadHandling.storeActualGamepadValuesAsCurrentGamepads();

            //Update Gyro values
            Robot.getInstance().getGyro().UpdateGyro(Robot.getInstance().getTeleOpRuntime());

            //Process the Driver Controls
            GamepadHandling.DriverControls();

            //Process the Operator Controls
            GamepadHandling.OperatorControls();

            //Look for AprilTags
            Robot.getInstance().getVision().LookForAprilTags();

            //Drive the Robot (manual if driver controls are active - or automatically if flag set)
            Robot.getInstance().getDriveTrain().drive();

            //Add AprilTag Telemetry
            Robot.getInstance().getVision().telemetryAprilTag();

            telemetry.update();

        }
        Robot.getInstance().getVision().getVisionPortal().close();
    }
}
