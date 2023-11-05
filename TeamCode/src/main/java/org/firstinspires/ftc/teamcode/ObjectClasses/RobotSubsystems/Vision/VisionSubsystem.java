package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.*;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

public final class VisionSubsystem extends SubsystemBase {

    public static VisionSubsystem.TunableVisionConstants tunableVisionConstants = new VisionSubsystem.TunableVisionConstants();

    public static class TunableVisionConstants {
        // Adjust these numbers to suit your robot.
        public double DESIRED_DISTANCE = 10; //  this is how close the camera should get to the target for alignment (inches)
        public double DESIRED_DISTANCE_SAFETY = 28; //  this is how close the camera should get to the target for safety(inches)

        //this is the tolerance before we rumble if vision is seeing things that are close
        final double PERCENT_TOLERANCE = 2;

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public double SAFETY_SPEED_GAIN = 0.01;   //
        public double STRAFE_GAIN = -0.025;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public double TURN_GAIN = -0.025;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        public double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
        public double MAX_AUTO_STRAFE = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
        public double MAX_AUTO_TURN = 0.6;   //  Clip the turn speed to this max value (adjust for your robot)

        public double MAX_MANUAL_BACKDROP_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    }
    private int blueTagFrameCount;
    private int redTagFrameCount;

    private static VisionPortal visionPortal;               // Used to manage the video source.
    private static AprilTagProcessor aprilTagProcessor;     // Used for managing the AprilTag detection process.
    private static InitVisionProcessor initVisionProcessor; // Used for managing detection of 1) team prop; 2) Alliance Color; and 3) Side of Field
    private Telemetry telemetry;
    private LinearOpMode activeOpMode;
    private MecanumDriveMona mecanumDrive;

    public void periodic()
    {

    }

    public void SwitchToAprilTagProcessor() {
        visionPortal.setProcessorEnabled(this.getInitVisionProcessor(), false);
        visionPortal.setProcessorEnabled(this.getAprilTagProcessor(), true);
    }

    public void SwitchToInitVisionProcessor() {
        visionPortal.setProcessorEnabled(this.getInitVisionProcessor(), true);
        visionPortal.setProcessorEnabled(this.getAprilTagProcessor(), false);
    }

    public void setStartingPose(InitVisionProcessor.AllianceColor allianceColor, InitVisionProcessor.SideOfField sideOfField) {
        if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE){
            mecanumDrive.pose = BLUE_BACKSTAGE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.BLUE && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE){
            mecanumDrive.pose = BLUE_AUDIENCE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.BACKSTAGE){
            mecanumDrive.pose = RED_BACKSTAGE_START_POSE;
        } else if (allianceColor == InitVisionProcessor.AllianceColor.RED && sideOfField == InitVisionProcessor.SideOfField.AUDIENCE){
            mecanumDrive.pose = RED_AUDIENCE_START_POSE;
        }
    }

    public enum AprilTagID {
        BLUE_BACKDROP_LEFT_TAG(1),
        BLUE_BACKDROP_CENTER_TAG(2),
        BLUE_BACKDROP_RIGHT_TAG(3),
        RED_BACKDROP_LEFT_TAG(4),
        RED_BACKDROP_CENTER_TAG(5),
        RED_BACKDROP_RIGHT_TAG(6),
        RED_AUDIENCE_WALL_LARGE_TAG(7),
        RED_AUDIENCE_WALL_SMALL_TAG(8),
        BLUE_AUDIENCE_WALL_SMALL_TAG(9),
        BLUE_AUDIENCE_WALL_LARGE_TAG(10);

        private final int id;
        private boolean isDetected;
        private AprilTagDetection detection;

        AprilTagID(int id) {
            this.id = id;
            this.isDetected = false;
            this.detection = null;
        }

        public static AprilTagID getByID(int id) {
            for (AprilTagID tag : values()) {
                if (tag.id == id)
                    return tag;
            }
            return null;
        }

        public static void setAllUnDetected() {
            for (AprilTagID tag : values()) {
                tag.isDetected = false;
            }
        }

        public void setDetected() {
            this.isDetected = true;
        }

        public void storeDetection(AprilTagDetection detect) {
            this.detection = detect;
        }
    }

    public enum DeliverLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum DeliverHeight {
        LOW,
        MID,
        HIGH
    }

    private DeliverLocation deliverLocationBlue = DeliverLocation.CENTER;
    private DeliverLocation deliverLocationRed = DeliverLocation.CENTER;

    private DeliverHeight deliverHeight = DeliverHeight.LOW;

    public boolean blueBackdropAprilTagFound = false;
    public boolean redBackdropAprilTagFound = false;

    public VisionSubsystem(final HardwareMap hMap, final String name) {
        // Create the vision processing during Init Period so we can find out Alliance Color, Side of Field, and Team Prop Location
        initVisionProcessor = new InitVisionProcessor();

        // Create the AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //these are very bad
                // ... these parameters are fx, fy, cx, cy.
                //.setLensIntrinsics(1394.6027293299926, 1394.6027293299926, 995.588675691456, 599.3212928484164)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hMap.get(WebcamName.class, name))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(initVisionProcessor)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void init() {
        telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        // During Init the AprilTag processor is off
        visionPortal.setProcessorEnabled(initVisionProcessor, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);

        telemetry.addLine("AprilTag on? " + visionPortal.getProcessorEnabled(aprilTagProcessor) + "       initVisionProcessor on? " + visionPortal.getProcessorEnabled(initVisionProcessor));
        telemetry.addLine("");

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        redTagFrameCount=0;
        blueTagFrameCount=0;

    }

    public InitVisionProcessor getInitVisionProcessor() {
        return initVisionProcessor;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }


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
            TelemetryPacket p = new TelemetryPacket();
            p.addLine("Camera is ready");
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!Robot.getInstance().getActiveOpMode().isStopRequested()) {
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

                double rangeError = (currentTag.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE_SAFETY);

                // Pick whichever value is lower
                double manualDriveLimit = Math.min(rangeError * tunableVisionConstants.SAFETY_SPEED_GAIN, tunableVisionConstants.MAX_MANUAL_BACKDROP_SPEED);
                if (manualDriveLimit < DriveSubsystem.driveParameters.safetyDriveSpeedFactor) {
                    DriveSubsystem.driveParameters.safetyDriveSpeedFactor = manualDriveLimit;
                }
            }
        }

        //If no april tags are detected then reset the safety drive speed factor
        if (currentDetections.size() == 0) {
            DriveSubsystem.driveParameters.safetyDriveSpeedFactor = DriveSubsystem.driveParameters.DRIVE_SPEED_FACTOR;
        }

        blueBackdropAprilTagFound = CheckBlueBackdropAprilTags();
        redBackdropAprilTagFound = CheckRedBackdropAprilTags();
    }

    private boolean CheckBlueBackdropAprilTags() {
        if (BLUE_BACKDROP_LEFT_TAG.isDetected || BLUE_BACKDROP_RIGHT_TAG.isDetected || BLUE_BACKDROP_CENTER_TAG.isDetected) {
            blueTagFrameCount++;
        } else
        {
            blueTagFrameCount=0;
        }

        //this method only returns true if we see 1 frame of one of the tags being detected
        if (blueTagFrameCount>0)
        {
            return true;
        } return false;
    }

    private boolean CheckRedBackdropAprilTags() {
        if (RED_BACKDROP_LEFT_TAG.isDetected || RED_BACKDROP_RIGHT_TAG.isDetected || RED_BACKDROP_CENTER_TAG.isDetected) {
            redTagFrameCount++;
        } else
        {
            redTagFrameCount=0;
        }

        //this method only returns true if we see 1 frame of one of the tags being detected
        if (redTagFrameCount>0)
        {
            return true;
        } return false;
    }

    public void DriveToRedAudienceWallTag() {
        if (Robot.getInstance().getActiveOpMode().gamepad1.left_bumper && (RED_AUDIENCE_WALL_LARGE_TAG.isDetected || RED_AUDIENCE_WALL_SMALL_TAG.isDetected)) {
            //if we can see the small april tag use that for navigation
            if (RED_AUDIENCE_WALL_SMALL_TAG.isDetected && RED_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.range < 35) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
                double headingError = RED_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.bearing;
                double yawError = RED_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

                telemetry.addData("Auto to Small Red", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (RED_AUDIENCE_WALL_LARGE_TAG.isDetected) // use the large tag until we can see the small tag
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (RED_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
                double headingError = RED_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.bearing;
                double yawError = RED_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.yaw;


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

                telemetry.addData("Auto to Large Red", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }
    }

    public void DriveToBlueAudienceWallTag() {
        if (Robot.getInstance().getActiveOpMode().gamepad1.right_bumper && (BLUE_AUDIENCE_WALL_LARGE_TAG.isDetected || BLUE_AUDIENCE_WALL_SMALL_TAG.isDetected)) {

            //if we can see the small april tag use that for navigation
            if (BLUE_AUDIENCE_WALL_SMALL_TAG.isDetected && BLUE_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.range < 35) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
                double headingError = BLUE_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.bearing;
                double yawError = BLUE_AUDIENCE_WALL_SMALL_TAG.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the rtunableVisionConstants.obot to move.
                double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

                telemetry.addData("Auto to Small Blue", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            } else if (BLUE_AUDIENCE_WALL_LARGE_TAG.isDetected)// use the large tag until we can see the small tag
            {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (BLUE_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
                double headingError = BLUE_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.bearing;
                double yawError = BLUE_AUDIENCE_WALL_LARGE_TAG.detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError *  tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
                Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

                telemetry.addData("Auto to Large Blue", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
        }
    }


    //  If right tag is detected AND one of the following three things is true, then drive to the right tag:
    //  1) the delivery location is right; or
    //  2) the delivery location is center, but center isn't found
    //  3) the delivery location is left, but left isn't found
    public void AutoDriveToBackdropBlue() {
        if (        BLUE_BACKDROP_RIGHT_TAG.isDetected && (
                    getDeliverLocationBlue().equals(DeliverLocation.RIGHT)    ||
                    (getDeliverLocationBlue().equals(DeliverLocation.CENTER) && !BLUE_BACKDROP_CENTER_TAG.isDetected)       ||
                    (getDeliverLocationBlue().equals(DeliverLocation.LEFT) && !BLUE_BACKDROP_LEFT_TAG.isDetected)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.bearing;
            double yawError = BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            telemetry.addData("Auto to Right Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, BLUE_BACKDROP_RIGHT_TAG);

        }
        //Drive to the Center backdrop if its the deliver location, or
        // if its left, but left is not detected or
        // if its right, but right is not detected

        else if (   BLUE_BACKDROP_CENTER_TAG.isDetected && (
                    getDeliverLocationBlue().equals(DeliverLocation.CENTER) ||
                    (getDeliverLocationBlue().equals(DeliverLocation.LEFT) && !BLUE_BACKDROP_LEFT_TAG.isDetected) ||
                    (getDeliverLocationBlue().equals(DeliverLocation.RIGHT) && !BLUE_BACKDROP_RIGHT_TAG.isDetected)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.bearing;
            double yawError = BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            // set the drive/turn strafe values for AutoDriving
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, BLUE_BACKDROP_CENTER_TAG);
            telemetry.addData("Auto to Center Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

        //Drive to the Right backdrop if its the deliver location, or
        // if its left, but left is not detected or
        // if its center, but center is not detected
        else if (   BLUE_BACKDROP_LEFT_TAG.isDetected &&
                (getDeliverLocationBlue().equals(DeliverLocation.LEFT)    ||
                (getDeliverLocationBlue().equals(DeliverLocation.RIGHT) && !BLUE_BACKDROP_RIGHT_TAG.isDetected)       ||
                (getDeliverLocationBlue().equals(DeliverLocation.CENTER) && !BLUE_BACKDROP_CENTER_TAG.isDetected)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.bearing;
            double yawError = BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, BLUE_BACKDROP_LEFT_TAG);

            telemetry.addData("Auto to Left Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

    }

    public void AutoDriveToBackdropRed() {

        if (    RED_BACKDROP_LEFT_TAG.isDetected && (
                getDeliverLocationRed().equals(DeliverLocation.LEFT) ||
                (getDeliverLocationRed().equals(DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)    ||
                (getDeliverLocationRed().equals(DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (RED_BACKDROP_LEFT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.bearing;
            double yawError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, RED_BACKDROP_LEFT_TAG);

            telemetry.addData("Auto to Left Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else if (     RED_BACKDROP_CENTER_TAG.isDetected && (
                        getDeliverLocationRed().equals(DeliverLocation.CENTER) ||
                       (getDeliverLocationRed().equals(DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                       (getDeliverLocationRed().equals(DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected))
        )
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (RED_BACKDROP_CENTER_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.bearing;
            double yawError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, RED_BACKDROP_CENTER_TAG);

            telemetry.addData("Auto to Center Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else if ( RED_BACKDROP_RIGHT_TAG.isDetected && (
                 getDeliverLocationRed().equals(DeliverLocation.RIGHT) ||
                (getDeliverLocationRed().equals(DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                (getDeliverLocationRed().equals(DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)))
        {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (RED_BACKDROP_RIGHT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            double headingError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.bearing;
            double yawError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.yaw;

            double drive = Range.clip(rangeError * tunableVisionConstants.SPEED_GAIN, -tunableVisionConstants.MAX_AUTO_SPEED, tunableVisionConstants.MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * tunableVisionConstants.TURN_GAIN, -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * tunableVisionConstants.STRAFE_GAIN, -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            resetRobotPoseBasedOnAprilTag(drive, strafe, turn, RED_BACKDROP_RIGHT_TAG);

            telemetry.addData("Auto to Right Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
    }


    public void setDeliverLocation(DeliverLocation location) {
        deliverLocationRed = location;
        deliverLocationBlue = location;
    }

    public void setDeliverHeight(DeliverHeight height) {
        deliverHeight = height;
    }

    public DeliverLocation getDeliverLocationRed() {
        return deliverLocationRed;
    }
    public DeliverLocation getDeliverLocationBlue() {
        return deliverLocationBlue;
    }

    public DeliverLocation getDeliverLocation(){
        if (initVisionProcessor.allianceColor == InitVisionProcessor.AllianceColor.RED)
        {
            return deliverLocationRed;
        } else return deliverLocationBlue;
    }


    private void resetRobotPoseBasedOnAprilTag(double drive, double strafe, double turn, AprilTagID tag) {

        //We have found the target if this is true
        if ((Math.abs(drive) < .13) && (Math.abs(strafe) < .13) && (Math.abs(turn) <.13)){
            VectorF tagVector = tag.detection.metadata.fieldPosition;
            double tagPosXOnField = tagVector.get(0);
            double tagPosYOnField = tagVector.get(1);
            Vector2d tagVector2D = new Vector2d(tagPosXOnField, tagPosYOnField);
            Vector2d distanceVector = new Vector2d(tag.detection.ftcPose.y,tag.detection.ftcPose.x);
            Vector2d result = new Vector2d(tagVector2D.x-distanceVector.x, tagVector2D.y-distanceVector.y);
            //TODO  need to change the facing here based on metadata to make this generic
            Pose2d realPose = new Pose2d(result.x, result.y, FACE_TOWARD_BACKSTAGE);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.pose = realPose;
            telemetry.addData("New Pose", "X %5.2f, Y %5.2f, heading %5.2f ", realPose.position.x, realPose.position.y, realPose.heading.real);
        }
    }

}

