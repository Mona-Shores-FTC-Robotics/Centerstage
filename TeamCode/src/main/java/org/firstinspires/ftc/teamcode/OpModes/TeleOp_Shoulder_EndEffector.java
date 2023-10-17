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

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Shoulder;

@TeleOp(name="TeleOp_Shoulder_EndEffector")
public class TeleOp_Shoulder_EndEffector extends LinearOpMode
{

    /** Create the robot **/
    Robot robot = Robot.createInstance(this);

    @Override public void runOpMode()
    {
        //Set the type of Robot
        RobotConstants.setRobot(RobotConstants.RobotType.ROBOT_SHOULDER_END_EFFECTOR);

        //Initialize the Robot
        robot.initialize(robot.getHardwareMap());

        telemetry = robot.getActiveOpMode().telemetry;

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

            //for testing leaving these buttons out here, but once we move to real code we can put them in the OperatorControls()
            //Don't forget its operator controlling these test buttons!
            if (GamepadHandling.operatorButtonPressed("x")) {
                Robot.getInstance().getEndEffector().GrabTwoPixels();
                telemetry.addLine("end effector 1");
                String name = Robot.getInstance().getShoulder().shoulder.getDeviceName();
                double pos = Robot.getInstance().getShoulder().shoulder.getPosition();

                telemetry.addData("name", name);
                telemetry.addData("pos", pos);
            }

            if (GamepadHandling.operatorButtonPressed("y")) {
                Robot.getInstance().getEndEffector().ReleaseBothPixels();
                telemetry.addLine("end effector 0");
                String name = Robot.getInstance().getShoulder().shoulder.getDeviceName();
                double pos = Robot.getInstance().getShoulder().shoulder.getPosition();

                telemetry.addData("name", name);
                telemetry.addData("pos", pos);
            }

            if (GamepadHandling.getCurrentOperatorGamepad().a) {
                Robot.getInstance().getShoulder().Rotate(Shoulder.ShoulderPositions.INTAKE);
                telemetry.addLine("shoulder 1");
                String name = Robot.getInstance().getShoulder().shoulder.getDeviceName();
                double pos = Robot.getInstance().getShoulder().shoulder.getPosition();

                telemetry.addData("name", name);
                telemetry.addData("pos", pos);
            }

            if (GamepadHandling.getCurrentOperatorGamepad().b) {
                Robot.getInstance().getShoulder().Rotate(Shoulder.ShoulderPositions.BACKDROP);
                telemetry.addLine("shoulder 0");
                String name = Robot.getInstance().getShoulder().shoulder.getDeviceName();
                double pos = Robot.getInstance().getShoulder().shoulder.getPosition();

                telemetry.addData("name", name);
                telemetry.addData("pos", pos);
            }
            telemetry.update();
        }
        robot.getVision().getVisionPortal().close();
        telemetry.clearAll();
    }
}
