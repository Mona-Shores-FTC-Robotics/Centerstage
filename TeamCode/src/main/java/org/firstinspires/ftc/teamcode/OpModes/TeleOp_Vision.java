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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Vision;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{
    Robot robot = Robot.createInstance(this);
    VisionPortal visionPortal;
    DriveTrain driveTrain;
    Gyro gyro;

    //Set defaults in case vision doesn't work
    private InitVisionProcessor.TeamPropLocation teamPropLocationAfterInit = InitVisionProcessor.TeamPropLocation.CENTER;
    private InitVisionProcessor.AllianceColor allianceColorAfterInit = InitVisionProcessor.AllianceColor.BLUE;
    private InitVisionProcessor.SideOfField sideOfFieldAfterInit = InitVisionProcessor.SideOfField.BACKSTAGE;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode()
    {
        Constants.setRobot(Constants.RobotType.ROBOT_VISION);
        robot.initialize(Robot.getInstance().getHardwareMap());
        visionPortal = Robot.getInstance().getVision().getVisionPortal();
        driveTrain = Robot.getInstance().getDriveTrain();
        gyro = Robot.getInstance().getGyro();
//        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeInInit()) {
            //Even though this is a teleop mode, this will test vision for now - this will need to be changed later
            teamPropLocationAfterInit = robot.getVision().getInitVisionProcessor().getTeamPropLocationFinal();
            allianceColorAfterInit = robot.getVision().getInitVisionProcessor().getAllianceColorFinal();
            sideOfFieldAfterInit =  robot.getVision().getInitVisionProcessor().getSideOfField();

            telemetry.addData("Alliance Color", robot.getVision().getInitVisionProcessor().getAllianceColorFinal());
            telemetry.addData("Side of the Field", robot.getVision().getInitVisionProcessor().getSideOfField());
            telemetry.addData("Team Prop Location", robot.getVision().getInitVisionProcessor().getTeamPropLocationFinal());
            telemetry.addData("Left Square Blue/Red Percent", JavaUtil.formatNumber(robot.getVision().getInitVisionProcessor().getLeftPercent(), 4, 1));
            telemetry.addData("Middle Square Blue/Red Percent", JavaUtil.formatNumber(robot.getVision().getInitVisionProcessor().getCenterPercent(), 4, 1));
            telemetry.addData("Right Square Blue/Red Percent", JavaUtil.formatNumber(robot.getVision().getInitVisionProcessor().getRightPercent(), 4, 1));
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("Team Prop Location After Init", teamPropLocationAfterInit);
        telemetry.addData("Alliance Color After Init", allianceColorAfterInit);
        telemetry.addData("Side of Field After Init", sideOfFieldAfterInit);
        telemetry.update();

        // After Init switch the vision processing to Apriltags
        visionPortal.setProcessorEnabled(robot.getVision().getInitVisionProcessor(), false);
        visionPortal.setProcessorEnabled(robot.getVision().getAprilTagProcessor(), true);

        runtime.reset();

        while (opModeIsActive())
        {
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            //Update Gyro values
            gyro.UpdateGyro(runtime);

            /** Driver Controls**/
            //Start button toggles field oriented control
            if(currentGamepad1.start && !previousGamepad1.start){
                if (driveTrain.getFieldOrientedControlFlag()) {
                    //drive normally - not in field oriented control
                    driveTrain.setFieldOrientedControlFlag(false);
                } else
                {
                    //drive in field oriented control
                    Robot.getInstance().getGyro().resetYaw();
                    driveTrain.setFieldOrientedControlFlag(true);
                }
            }


            /** Operator Controls**/
            // the X/Y/B buttons set the deliver location to left, center, or right
            if(currentGamepad2.x && !previousGamepad2.x){
               Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.LEFT);
            }
            if(currentGamepad2.y && !previousGamepad2.y){
                Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.CENTER);
            }
            if(currentGamepad2.b && !previousGamepad1.b){
                Robot.getInstance().getVision().setDeliverLocation(Vision.DeliverLocation.RIGHT);
            }


            // Look for April Tags
            Robot.getInstance().getVision().LookForAprilTags();

            // Drive the Robot (manual if driver controls are active - or automatically if flag set)
            Robot.getInstance().getDriveTrain().drive();

            // Add April Tag Telemetry
            Robot.getInstance().getVision().telemetryAprilTag();


            telemetry.addLine("Gyro Readings");
            telemetry.addLine("Yaw Angle in Degrees" + JavaUtil.formatNumber(gyro.getYawDegrees(), 4, 0));
            telemetry.update();

        }
        visionPortal.close();
    }
}
