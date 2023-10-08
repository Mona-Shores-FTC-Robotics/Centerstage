package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.Vision.AprilTagID.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor.*;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 15; //  this is how close the camera should get to the target for alignment (inches)
    final double DESIRED_DISTANCE_SAFETY = 30; //  this is how close the camera should get to the target for safety(inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.04  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  -0.037 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  -0.03  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.9;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.9;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.7;   //  Clip the turn speed to this max value (adjust for your robot)

    final double MAX_MANUAL_BACKDROP_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;     // Used for managing the AprilTag detection process.
    private InitVisionProcessor initVisionProcessor; // Used for managing detection of 1) team prop; 2) Alliance Color; and 3) Side of Field
    private Telemetry telemetry;
    private LinearOpMode activeOpMode;

    public void SwitchToAprilTagProcessor() {
        visionPortal.setProcessorEnabled(this.getInitVisionProcessor(), false);
        visionPortal.setProcessorEnabled(this.getAprilTagProcessor(), true);
    }

    public void SwitchToInitVisionProcessor() {
        visionPortal.setProcessorEnabled(this.getInitVisionProcessor(), true);
        visionPortal.setProcessorEnabled(this.getAprilTagProcessor(), false);
    }

    public enum AprilTagID {
        BLUE_BACKDROP_LEFT (1),
        BLUE_BACKDROP_CENTER (2),
        BLUE_BACKDROP_RIGHT (3),
        RED_BACKDROP_LEFT (4),
        RED_BACKDROP_CENTER (5),
        RED_BACKDROP_RIGHT (6),
        RED_AUDIENCE_WALL_LARGE (7),
        RED_AUDIENCE_WALL_SMALL (8),
        BLUE_AUDIENCE_WALL_SMALL (9),
        BLUE_AUDIENCE_WALL_LARGE (10);

        private final int id;
        private boolean isDetected;
        private AprilTagDetection detection;

        AprilTagID(int id) {
            this.id = id;
            this.isDetected = false;
            this.detection = null;
        }

        public static AprilTagID getByID(int id){
            for (AprilTagID tag : values()){
                if(tag.id == id)
                    return tag;
            }
            return null;
        }

        public static void setAllUnDetected()
        {
            for (AprilTagID tag : values()){
                tag.isDetected = false;
            }
        }

        public void setDetected()
        {
            this.isDetected = true;
        }

        public void storeDetection(AprilTagDetection detect)
        {
            this.detection = detect;
        }
    }

    public enum DeliverLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    private DeliverLocation deliverLocationBlue = DeliverLocation.RIGHT;
    private DeliverLocation deliverLocationRed = DeliverLocation.LEFT;



    public boolean blueBackdropAprilTagFound=false;
    public boolean redBackdropAprilTagFound=false;

    public Vision() {

    }

    public void init() {

        telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        // Initialize the vision processing during Init Period so we can find out Alliance Color, Side of Field, and Team Prop Location
        initVisionProcessor = new InitVisionProcessor();

        // Initialize the AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // ... these parameters are fx, fy, cx, cy.
                //.setLensIntrinsics(1394.6027293299926, 1394.6027293299926, 995.588675691456, 599.3212928484164)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(Robot.getInstance().getHardwareMap().get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640 , 480))
                .enableLiveView(true)
                .addProcessor(initVisionProcessor)
                .addProcessor(aprilTagProcessor)
                .build();

        // During Init the AprilTag processor is off
        visionPortal.setProcessorEnabled(initVisionProcessor, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);

        telemetry.addLine("AprilTag on? " + visionPortal.getProcessorEnabled(aprilTagProcessor) + "       initVisionProcessor on? " + visionPortal.getProcessorEnabled(initVisionProcessor));
        telemetry.addLine("");

        setManualExposure(7, 250);  // Use low exposure time to reduce motion blur
     }

    public InitVisionProcessor getInitVisionProcessor()
    {
        return initVisionProcessor;
    }
    public AprilTagProcessor getAprilTagProcessor() { return aprilTagProcessor;}


    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */
    private void setManualExposure(int exposureMS, int gain) {

        activeOpMode = Robot.getInstance().getActiveOpMode();

        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!activeOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                activeOpMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!Robot.getInstance().getActiveOpMode().isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                activeOpMode.sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            activeOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            activeOpMode.sleep(20);
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    public void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        myAprilTagDetections = aprilTagProcessor.getDetections();
        //myAprilTagDetections = aprilTagProcessor.getFreshDetections();

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
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void LookForAprilTags() {
        AprilTagID currentTag;

        //mark all the tags as undetected
        AprilTagID.setAllUnDetected();

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)) {
                currentTag = AprilTagID.getByID(detection.id);
                currentTag.setDetected();
                currentTag.storeDetection(detection);

                double rangeError = (currentTag.detection.ftcPose.range - DESIRED_DISTANCE_SAFETY);

                // Use this to limit drive speed based on distance
                double manualDriveLimit = Range.clip(rangeError * SPEED_GAIN, -MAX_MANUAL_BACKDROP_SPEED, MAX_MANUAL_BACKDROP_SPEED);
                if (manualDriveLimit < Robot.getInstance().getDrivetrain().getSafetyDriveSpeedFactor()) {
                    Robot.getInstance().getDrivetrain().setSafetyDriveSpeedFactor(manualDriveLimit);
                }
            }
        }

        if (currentDetections.size()==0)  Robot.getInstance().getDrivetrain().setSafetyDriveSpeedFactor(Robot.getInstance().getDrivetrain().DRIVE_SPEED_FACTOR);

        blueBackdropAprilTagFound = CheckBlueBackdropAprilTags();
        redBackdropAprilTagFound = CheckRedBackdropAprilTags();
    }

    private boolean CheckBlueBackdropAprilTags() {
        if (BLUE_BACKDROP_LEFT.isDetected || BLUE_BACKDROP_RIGHT.isDetected || BLUE_BACKDROP_CENTER.isDetected) return true;
        else return false;
    }
    private boolean CheckRedBackdropAprilTags() {
        if (RED_BACKDROP_LEFT.isDetected || RED_BACKDROP_RIGHT.isDetected || RED_BACKDROP_CENTER.isDetected) return true;
        else return false;
    }

    public void DriveToRedAudienceWallTag() {
        if (Robot.getInstance().getActiveOpMode().gamepad1.left_bumper && (RED_AUDIENCE_WALL_LARGE.isDetected || RED_AUDIENCE_WALL_SMALL.isDetected)) {
            //if we can see the small april tag use that for navigation
            if (RED_AUDIENCE_WALL_SMALL.isDetected && RED_AUDIENCE_WALL_SMALL.detection.ftcPose.range < 35) {
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (RED_AUDIENCE_WALL_SMALL.detection.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = RED_AUDIENCE_WALL_SMALL.detection.ftcPose.bearing;
                    double yawError = RED_AUDIENCE_WALL_SMALL.detection.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                    Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                    Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                    telemetry.addData("Auto to Small Red", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            else if (RED_AUDIENCE_WALL_LARGE.isDetected ) // use the large tag until we can see the small tag
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_AUDIENCE_WALL_LARGE.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_AUDIENCE_WALL_LARGE.detection.ftcPose.bearing;
                double yawError = RED_AUDIENCE_WALL_LARGE.detection.ftcPose.yaw;


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Large Red", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }
    }

    public void DriveToBlueAudienceWallTag() {
        if (Robot.getInstance().getActiveOpMode().gamepad1.right_bumper && (BLUE_AUDIENCE_WALL_LARGE.isDetected || BLUE_AUDIENCE_WALL_SMALL.isDetected)) {

            //if we can see the small april tag use that for navigation
            if (BLUE_AUDIENCE_WALL_SMALL.isDetected && BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.range < 35) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.bearing;
                double yawError = BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Small Blue", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            }
            else if (BLUE_AUDIENCE_WALL_LARGE.isDetected)// use the large tag until we can see the small tag
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_AUDIENCE_WALL_LARGE.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_AUDIENCE_WALL_LARGE.detection.ftcPose.bearing;
                double yawError = BLUE_AUDIENCE_WALL_LARGE.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Large Blue", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }
    }

    public void AutoDriveToBackdropBlue() {
        if (getDeliverLocationBlue().equals(DeliverLocation.RIGHT) && BLUE_BACKDROP_RIGHT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_RIGHT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_RIGHT.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_RIGHT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Right Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
          else if (getDeliverLocationBlue().equals(DeliverLocation.LEFT) && BLUE_BACKDROP_LEFT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_LEFT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_LEFT.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_LEFT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Left Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            else if (getDeliverLocationBlue().equals(DeliverLocation.CENTER) && BLUE_BACKDROP_CENTER.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_CENTER.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_CENTER.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_CENTER.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // set the drive/turn strafe values for AutoDriving
                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Center Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }

    public void AutoDriveToBackdropRed() {
         if (getDeliverLocationRed().equals(DeliverLocation.LEFT) && RED_BACKDROP_LEFT.isDetected  ) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_LEFT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_LEFT.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_LEFT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Left Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            else if (getDeliverLocationRed().equals(DeliverLocation.CENTER) && RED_BACKDROP_CENTER.isDetected){
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_CENTER.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_CENTER.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_CENTER.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Center Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            else if (getDeliverLocationRed().equals(DeliverLocation.RIGHT) && RED_BACKDROP_RIGHT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_RIGHT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_RIGHT.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_RIGHT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDrivetrain().setAprilTagDrive(drive);
                Robot.getInstance().getDrivetrain().setAprilTagStrafe(strafe);
                Robot.getInstance().getDrivetrain().setAprilTagTurn(turn);

                telemetry.addData("Auto to Right Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }


    public void setDeliverLocation(DeliverLocation d)
    {
        deliverLocationRed = d;
        deliverLocationBlue = d;
    }


    public DeliverLocation getDeliverLocationBlue()
    {
        return deliverLocationBlue;
    }
    public DeliverLocation getDeliverLocationRed()
    {
        return deliverLocationRed;
    }

    public void telemetryForInitProcessing() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        telemetry.addData("Alliance Color", Robot.getInstance().getVision().getInitVisionProcessor().getAllianceColorFinal());
        telemetry.addData("Side of the Field", Robot.getInstance().getVision().getInitVisionProcessor().getSideOfFieldFinal());
        telemetry.addData("Team Prop Location", Robot.getInstance().getVision().getInitVisionProcessor().getTeamPropLocationFinal());
        telemetry.addLine("");
        telemetry.addData("Left Square Blue/Red Percent", JavaUtil.formatNumber(getInitVisionProcessor().getLeftPercent(), 4, 1));
        telemetry.addData("Middle Square Blue/Red Percent", JavaUtil.formatNumber(getInitVisionProcessor().getCenterPercent(), 4, 1));
        telemetry.addData("Right Square Blue/Red Percent", JavaUtil.formatNumber(getInitVisionProcessor().getRightPercent(), 4, 1));
        telemetry.addData("Stage Door Left Percent", JavaUtil.formatNumber(getInitVisionProcessor().percentLeftStageDoorZone, 4, 1));
        telemetry.addData("Right Square Right Percent", JavaUtil.formatNumber(getInitVisionProcessor().percentRightStageDoorZone, 4, 1));
    }




}

