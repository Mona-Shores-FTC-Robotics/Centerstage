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

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.RED_BACKSTAGE_START_POSE;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.VisionDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@TeleOp(name="TeleOp_PostAuto")
public class TeleOp_PostAuto extends LinearOpMode
{
    @Override public void runOpMode()
    {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        //Create the Robot
        Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);

        //Initialize the Robot
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        /* Setup Button Bindings **/
        new VisionDriverBindings(   gamepadHandling.getDriverGamepad());

        while (opModeInInit()) {
            VisionTelemetry.telemetryForInitProcessing(gamepadHandling);
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = RED_BACKSTAGE_START_POSE;

        //Reset Gyro
        if (MatchConfig.autoHasRun=true)
        {
            MatchConfig.autoHasRun=false;
            //This method uses the values saved at the end of the auto to transition to teleop
            Robot.getInstance().getGyroSubsystem().postAutoGyroReset();
        }
        //assume the user is facing the backdrop
        else {
            //this will reset the gyro and set the relative yaw to the heading of the robot, which unless we set a start heading will be 0
            Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPose();
            //this will make the gyro reset the offset and pose the next time we get to the april tags.
            Robot.getInstance().getVisionSubsystem().resetHeading=true;
        }

        //Start the TeleOp Timer
        ElapsedTime teleOpTimer = new ElapsedTime();
        teleOpTimer.reset();

        while (opModeIsActive())
        {
            //Run the Scheduler
            CommandScheduler.getInstance().run();
            gamepadHandling.getDriverGamepad().readButtons();

            //Look for AprilTags
            Robot.getInstance().getVisionSubsystem().LookForAprilTags();

            //Add AprilTag Telemetry
            if (gamepad1.left_trigger>.1) {
                telemetry.addData("Alliance Color", MatchConfig.finalAllianceColor);
//                Robot.getInstance().getVisionSubsystem().telemetryAprilTag();
                Robot.getInstance().getGyroSubsystem().DriverStationTelemetry();
            }

            //Add DriveTrain Telemetry
            if (gamepad1.right_trigger>.1) {
                Robot.getInstance().getDriveSubsystem().DriverStationTelemetry();
                Robot.getInstance().getGyroSubsystem().DriverStationTelemetry();
                telemetry.addData("leftstick y",    gamepadHandling.getDriverGamepad().getLeftY());
                telemetry.addData("leftstick x",    gamepadHandling.getDriverGamepad().getLeftX() );
                telemetry.addData("rightstick x",    gamepadHandling.getDriverGamepad().getRightX());
            }

            telemetry.update();
        }
    }
}

