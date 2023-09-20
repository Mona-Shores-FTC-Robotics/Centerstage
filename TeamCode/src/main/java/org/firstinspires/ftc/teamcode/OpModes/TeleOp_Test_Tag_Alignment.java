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

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp_Test_Tag_Alignment")
public class TeleOp_Test_Tag_Alignment extends LinearOpMode
{
    Robot robot = Robot.createInstance(this);
    private DriveTrain MecDrive;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    TfodProcessor tfodProcessor;

    String StreamingStatus;
    String LiveViewStatus;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 10; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_BLUE_ID = 10;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int DESIRED_TAG_RED_ID = 7;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private AprilTagDetection desiredTagRed = null;     // Used to hold the data for a detected AprilTag
    private AprilTagDetection desiredTagBlue = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        float myFPS;
        robot.initialize(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // This OpMode turns RC LiveView and Streaming off and on.
        StreamingStatus = "ON";
        LiveViewStatus = "ON";

        // Initialize AprilTag before waitForStart.
        initAprilTag();

        // Get the current vision frame rate.
        myFPS = visionPortal.getFps();

        boolean targetBlueID10Found     = false;    // Set to true when an AprilTag target is detected
        boolean targetRedID7Found     = false;    // Set to true when an AprilTag target is detected
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

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        while (opModeInInit()) {
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addLine("");
            Toggle_and_Telemetry();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            Toggle_and_Telemetry();

            targetBlueID10Found = false;
            targetRedID7Found = false;
            desiredTagRed  = null;
            desiredTagBlue  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && ((detection.id == DESIRED_TAG_BLUE_ID))  ){
                    targetBlueID10Found = true;
                    desiredTagBlue = detection;
                }
                if ((detection.metadata != null) && ((detection.id == DESIRED_TAG_RED_ID))  ){
                    targetRedID7Found = true;
                    desiredTagRed = detection;
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetBlueID10Found) {
                telemetry.addData(">","HOLD Right-Bumper to Drive to Blue Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTagBlue.id, desiredTagBlue.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTagBlue.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTagBlue.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTagBlue.ftcPose.yaw);

            }

            if (targetRedID7Found) {
                telemetry.addData(">","HOLD Left-Bumper to Drive to Red Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTagRed.id, desiredTagRed.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTagRed.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTagRed.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTagRed.ftcPose.yaw);
            }

            if (!(targetBlueID10Found || targetRedID7Found)) {
                telemetry.addData(">","Drive using joystick to find target\n");
            }

            // If Right Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.right_bumper && targetBlueID10Found) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTagBlue.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTagBlue.ftcPose.bearing;
                double  yawError        = desiredTagBlue.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            else if (gamepad1.left_bumper && targetRedID7Found) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTagRed.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTagRed.ftcPose.bearing;
            double  yawError        = desiredTagRed.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            telemetry.update();
        }
        visionPortal.close();
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {

        // Create the TensorFlow Object Detection processor and assign it to a variable.
       tfodProcessor = TfodProcessor.easyCreateWithDefaults();
        // Next, create a VisionPortal.

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(false)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // ... these parameters are fx, fy, cx, cy.
                //.setLensIntrinsics(1394.6027293299926, 1394.6027293299926, 995.588675691456, 599.3212928484164)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
           }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640 , 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);
        builder.addProcessor(tfodProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    /**
     * Describe this function...
     */
    private void Toggle_and_Telemetry() {
        Toggle_LiveView_and_Streaming();
        Toggle_Processors();
        Frames_Per_Second();
        telemetryAprilTag();
        telemetryTfod();

        // Share the CPU.
        sleep(20);
    }

    /**
     * Describe this function...
     */
    private void Frames_Per_Second() {
        // Get the current vision frame rate.
        telemetry.addData("Frames Per Second (FPS)", Double.parseDouble(JavaUtil.formatNumber(visionPortal.getFps(), 0)));
        telemetry.addLine("");
    }



    /**
     * Describe this function...
     */
    private void Toggle_LiveView_and_Streaming() {
        if (gamepad1.dpad_left) {
            // Temporarily stop the live view (RC preview).
            visionPortal.stopLiveView();
            LiveViewStatus = "OFF";
        } else if (gamepad1.dpad_right) {
            // Start the live view (RC preview) again.
            visionPortal.resumeLiveView();
            LiveViewStatus = "ON";
        }
        if (gamepad1.dpad_down) {
            // Temporarily stop the streaming session. This can save CPU
            // resources, with the ability to resume quickly when needed.
            visionPortal.stopStreaming();
            StreamingStatus = "OFF";
        } else if (gamepad1.dpad_up) {
            // Resume the streaming session if previously stopped.
            visionPortal.resumeStreaming();
            StreamingStatus = "ON";
        }
        telemetry.addLine("Dpad L/R LiveView: " + LiveViewStatus + "    Down/Up Streaming: " + StreamingStatus);
        telemetry.addLine("");
    }

    /**
     * Describe this function...
     */
    private void Toggle_Processors() {
        if (gamepad1.x) {
            // Enable or disable the AprilTag processor.
            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        } else if (gamepad1.y) {
            // Enable or disable the AprilTag processor.
            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        }
        if (gamepad1.a) {
            // Enable or disable the TensorFlow Object Detection processor.
            visionPortal.setProcessorEnabled(tfodProcessor, true);
        } else if (gamepad1.b) {
            // Enable or disable the TensorFlow Object Detection processor.
            visionPortal.setProcessorEnabled(tfodProcessor, false);
        }
        // Get whether the AprilTag processor is enabled.
        // Get whether the TensorFlow Object Detection processor is enabled.
        telemetry.addLine("X/Y AprilTag on? " + visionPortal.getProcessorEnabled(aprilTagProcessor) + "       A/B TFOD on? " + visionPortal.getProcessorEnabled(tfodProcessor));
        telemetry.addLine("");
    }



    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        myAprilTagDetections = aprilTagProcessor.getDetections();
        // Alternate: get only fresh detections.
        // Get a list containing detections that were detected since the last call to this method, or null if no new detections are available. This is useful to avoid re-processing the same detections multiple times.
        // myAprilTagDetections = myAprilTagProcessor.getFreshDetections();

        if (myAprilTagDetections != null) {

            telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
            // Iterate through list and call a function to
            // display info for each recognized AprilTag.
            for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
                myAprilTagDetection = myAprilTagDetection_item;
                // Display info about the detection.
                telemetry.addLine("");
                if (myAprilTagDetection.metadata != null) {
                    telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                    telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                    telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + "  (deg)");
                    telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
                } else {
                    telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                    telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
                }
            } // end for() loop

        } // end if != null

        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        //myTfodRecognitions = tfodProcessor.getRecognitions();
        // Alternate: get only fresh recognitions.
        // Get a list containing recognitions that were detected since the last call to this method, or null if no new recognitions are available. This is useful to avoid re-processing the same recognitions multiple times.
        myTfodRecognitions = tfodProcessor.getFreshRecognitions();

        if (myTfodRecognitions != null) {

            telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition myTfodRecognition_item : myTfodRecognitions) {
                myTfodRecognition = myTfodRecognition_item;
                // Display info about the recognition.
                telemetry.addLine("");
                // Display label and confidence.
                // Display the label and confidence for the recognition.
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                // Display position.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                // Display the position of the center of the detection boundary for the recognition
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                // Display size
                // Display the size of detection boundary for the recognition
                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
            } // end for() loop

        } // end if != null

    }

}
