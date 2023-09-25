package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Vision.AprilTagID.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;     // Used for managing the AprilTag detection process.
    private InitVisionProcessor initVisionProcessor; // Used for managing detection of 1) team prop; 2) Alliance Color; and 3) Side of Field
    private Telemetry telemetry;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 15; //  this is how close the camera should get to the target (inches)

    private boolean autoDrive = false;

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

    private DeliverLocation deliverLocation = DeliverLocation.CENTER;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.047  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.018 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.06  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 2.5;   //  Clip the turn speed to this max value (adjust for your robot)

    public Vision() {

    }

    public void init() {
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        // Initialize the vision processing during Init Period so we can find out Alliance Color, Side of Field, and Team Prop Location
        initVisionProcessor = new InitVisionProcessor(telemetry);

        // Initialize the AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
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

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
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

        LinearOpMode activeOpMode =  Robot.getInstance().getActiveOpMode();

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
            }
        }

        // Tell driver when we see the large/small audience wall AprilTags - tell them they can hit the bumpers to drive to the small targets
       if (BLUE_AUDIENCE_WALL_LARGE.isDetected) {
            telemetry.addData(">", "HOLD Right-Bumper to Drive to Small Blue Target \n");
       }

       if (RED_AUDIENCE_WALL_LARGE.isDetected) {
            telemetry.addData(">", "HOLD Left-Bumper to Drive to Small Red Target\n");
       }

        autoDrive = false;
        Robot.getInstance().getDriveTrain().setBackdropSafetyZone(false);

        DriveToBlueAudienceWallTag();
        DriveToRedAudienceWallTag();

        if (!autoDrive) {
            AutoDriveToBackdropBlue();
            AutoDriveToBackdropRed();
        }

        if (autoDrive) {
            //set the manual control flag to false so ew know that we are doing automated driving
            Robot.getInstance().getDriveTrain().setManualDriveControlFlag(false);
        } else {
            //no april tag is being driven to (either because we didn't see them or the user didn't hold down the bumpers) - so we set manual drive control to true
            Robot.getInstance().getDriveTrain().setManualDriveControlFlag(true);
            Robot.getInstance().getDriveTrain().setBackdropSafetyZone(false);
            Robot.getInstance().getDriveTrain().setDriveSpeedFactor(1.0);
        }
    }

    private void DriveToRedAudienceWallTag() {
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

                    Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                    Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                    Robot.getInstance().getDriveTrain().setAutoTurn(turn);

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

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                telemetry.addData("Auto to Large Red", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            autoDrive = true;
        }
    }

    private void DriveToBlueAudienceWallTag() {
        if (Robot.getInstance().getActiveOpMode().gamepad1.right_bumper && (BLUE_AUDIENCE_WALL_LARGE.isDetected || BLUE_AUDIENCE_WALL_SMALL.isDetected)) {

            //if we can see the small april tag use that for navigation
            if (BLUE_AUDIENCE_WALL_LARGE.isDetected && BLUE_AUDIENCE_WALL_LARGE.detection.ftcPose.range < 35) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.bearing;
                double yawError = BLUE_AUDIENCE_WALL_SMALL.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

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

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                telemetry.addData("Auto to Large Blue", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            autoDrive = true;
        }
    }

    private void AutoDriveToBackdropBlue() {
        if (BLUE_BACKDROP_LEFT.isDetected || BLUE_BACKDROP_CENTER.isDetected || BLUE_BACKDROP_RIGHT.isDetected) {
            Robot.getInstance().getDriveTrain().setBackdropSafetyZone(true);
            //if we can see the middle april tag use that for navigation
            if (getDeliverLocation().equals(DeliverLocation.CENTER) && BLUE_BACKDROP_CENTER.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_CENTER.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_CENTER.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_CENTER.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // set the drive/turn strafe values for AutoDriving
                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
//                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Center Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (getDeliverLocation().equals(DeliverLocation.LEFT) && BLUE_BACKDROP_LEFT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_LEFT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_LEFT.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_LEFT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Left Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            else if (getDeliverLocation().equals(DeliverLocation.RIGHT) && BLUE_BACKDROP_RIGHT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_BACKDROP_RIGHT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = BLUE_BACKDROP_RIGHT.detection.ftcPose.bearing;
                double yawError = BLUE_BACKDROP_RIGHT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Right Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            autoDrive = true;
        }
    }



    private void AutoDriveToBackdropRed() {
        if (RED_BACKDROP_LEFT.isDetected || RED_BACKDROP_CENTER.isDetected || RED_BACKDROP_RIGHT.isDetected) {
            Robot.getInstance().getDriveTrain().setBackdropSafetyZone(true);
            //if we can see the middle april tag use that for navigation
            if (getDeliverLocation().equals(DeliverLocation.CENTER) && RED_BACKDROP_CENTER.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_CENTER.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_CENTER.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_CENTER.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Center Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (getDeliverLocation().equals(DeliverLocation.LEFT) && RED_BACKDROP_LEFT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_LEFT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_LEFT.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_LEFT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Left Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            else if (getDeliverLocation().equals(DeliverLocation.RIGHT) && RED_BACKDROP_RIGHT.isDetected) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_BACKDROP_RIGHT.detection.ftcPose.range - DESIRED_DISTANCE);
                double headingError = RED_BACKDROP_RIGHT.detection.ftcPose.bearing;
                double yawError = RED_BACKDROP_RIGHT.detection.ftcPose.yaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveTrain().setAutoDrive(drive);
                Robot.getInstance().getDriveTrain().setAutoStrafe(strafe);
                Robot.getInstance().getDriveTrain().setAutoTurn(turn);

                // set the noCrashDriveSpeedFactor for if the driver takes over control near the backdrop
                Robot.getInstance().getDriveTrain().setDriveSpeedFactor(drive);

                telemetry.addData("Auto to Right Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            autoDrive = true;
        }
    }


    public void setDeliverLocation(DeliverLocation d)
    {
        deliverLocation = d;
    }

    public DeliverLocation getDeliverLocation()
    {
        return deliverLocation;
    }


    private void tagTelemetry(AprilTagID tag)
    {
        if (tag.isDetected) {
            telemetry.addData("Target", "ID %d (%s)", tag.id, tag.detection.metadata.name);
            telemetry.addData("Range", "%5.1f inches", tag.detection.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", tag.detection.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", tag.detection.ftcPose.yaw);
    }
}



}
