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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="TeleOp_Vision")
public class TeleOp_Vision extends LinearOpMode
{
    Robot robot = Robot.createInstance(this);
    VisionPortal visionPortal;

    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel


    private double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    private double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    private double  turn            = 0;        // Desired turning power/speed (-1 to +1)


    @Override public void runOpMode()
    {
        Constants.setRobot(Constants.RobotType.ROBOT_VISION); //This OpMode uses the robot with a Chassis and a Camera
        robot.initialize(hardwareMap);
        visionPortal = Robot.getInstance().getVision().getVisionPortal();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = robot.getDriveTrain().driveMotor[0]; //LFDrive
        rightFrontDrive = robot.getDriveTrain().driveMotor[1]; //RFDrive
        leftBackDrive  = robot.getDriveTrain().driveMotor[2]; //LBDrive
        rightBackDrive = robot.getDriveTrain().driveMotor[3]; //RBDrive

        while (opModeInInit()) {
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addLine("");

            Robot.getInstance().getVision().Frames_Per_Second();

            telemetry.update();
        }
        waitForStart();

        // After Init switch the vision processing to Apriltags
        visionPortal.setProcessorEnabled(robot.getVision().getInitVisionProcessor(), false);
        visionPortal.setProcessorEnabled(robot.getVision().getAprilTagProcessor(), true);

        while (opModeIsActive())
        {
            // Look for April Tags
            Robot.getInstance().getVision().LookForAprilTags();

            // Drive the Robot (manual if driver controls are active - or automatically if flag set)
            Robot.getInstance().getDriveTrain().drive();

            // Add April Tag Telemetry
            Robot.getInstance().getVision().telemetryAprilTag();

            telemetry.update();

        }
        visionPortal.close();
    }
}
