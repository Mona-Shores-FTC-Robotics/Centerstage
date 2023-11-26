package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.CENTER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.RIGHT;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
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
@Config
public final class VisionSubsystem extends SubsystemBase {

    public static VisionSubsystem.TunableVisionConstants tunableVisionConstants = new VisionSubsystem.TunableVisionConstants();

    public static class TunableVisionConstants {
        // Adjust these numbers to suit your robot.
        public double DESIRED_DISTANCE = 18; //  this is how close the camera should get to the target for alignment (inches)
        public double DESIRED_DISTANCE_SAFETY = 28; //  this is how close the camera should get to the target for safety(inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public double SAFETY_SPEED_GAIN = 0.01;   //

        public double RANGE_GAIN = 0.05;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public double STRAFE_GAIN = .165;   // As we lose power this needs to go up..  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public double TURN_GAIN = .05;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        public double RANGE_FEEDFORWARD =.03; //this seems to be the amount to move forard needed
        public double STRAFE_FEEDFORWARD=.00185; //this is about right for strafe feedfoward
        public double TURN_FEEDFORWARD=.05;

        public double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
        public double MAX_AUTO_STRAFE = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
        public double MAX_AUTO_TURN = 0.45;   //  Clip the turn speed to this max value (adjust for  your robot)

        public double MAX_MANUAL_BACKDROP_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public double BACKDROP_DRIVE_THRESHOLD=2;
        public double BACKDROP_STRAFE_THRESHOLD=2;
        public double BACKDROP_TURN_THRESHOLD=2;
        public double APRIL_TAG_LAST_SEEN_THRESHOLD_SEC = .4;
        public int BACKDROP_POSE_COUNT_THRESHOLD=5;
    }

    public double rangeError;
    public double xError;
    public double yawError;
    public double bearingError;
    private int backdropPoseCount=0;
    public boolean resetPoseReady=false;
    public boolean resetHeading=false;
    public Pose2d resetPose;

    private boolean recentBlueLeft=false;
    private boolean recentBlueCenter=false;
    private boolean recentBlueRight=false;

    private boolean recentRedLeft=false;
    private boolean recentRedCenter=false;
    private boolean recentRedRight=false;

    private AprilTagID deliveryTag;
    private AprilTagID offsetTagToLeft;
    private AprilTagID offsetTagToRight;


    private static VisionPortal visionPortal;               // Used to manage the video source.
    private static AprilTagProcessor aprilTagProcessor;     // Used for managing the AprilTag detection process.
    private static InitVisionProcessor initVisionProcessor; // Used for managing detection of 1) team prop; 2) Alliance Color; and 3) Side of Field
    private Telemetry telemetry;
    private LinearOpMode activeOpMode;
    private MecanumDriveMona mecanumDrive;

    public void periodic()
    {

        MatchConfig.telemetryPacket.put("Delivery Location: ", Robot.getInstance().getVisionSubsystem().getDeliverLocation() );
    }

    public void SwitchToAprilTagProcessor() {
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getInitVisionProcessor(), false);
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getAprilTagProcessor(), true);
    }

    public void SwitchToInitVisionProcessor() {
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getInitVisionProcessor(), true);
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getAprilTagProcessor(), false);
    }

    public void VisionOff() {
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getInitVisionProcessor(), false);
        Robot.getInstance().getVisionSubsystem().getVisionPortal().setProcessorEnabled(this.getAprilTagProcessor(), false);
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
        private double timestamp;
        private AprilTagDetection detection;

        AprilTagID(int id) {
            this.id = id;
            this.isDetected = false;
            this.detection = null;
            this.timestamp=0;
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

        public void setTimestamp() {
           this.timestamp = MatchConfig.timestampTimer.seconds();
        }

        public double getTimestamp() {
            return this.timestamp;
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

    private DeliverLocation deliverLocation = DeliverLocation.CENTER;

    public boolean blueBackdropAprilTagFoundRecently = false;
    public boolean redBackdropAprilTagFoundRecently = false;

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
        initVisionProcessor= Robot.getInstance().getVisionSubsystem().getInitVisionProcessor();
        aprilTagProcessor = Robot.getInstance().getVisionSubsystem().getAprilTagProcessor();
        visionPortal = Robot.getInstance().getVisionSubsystem().getVisionPortal();

        // During Init the AprilTag processor is off
        visionPortal.setProcessorEnabled(initVisionProcessor, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);

        telemetry.addLine("AprilTag on? " + visionPortal.getProcessorEnabled(aprilTagProcessor) + "       initVisionProcessor on? " + visionPortal.getProcessorEnabled(initVisionProcessor));
        telemetry.addLine("");

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        recentBlueLeft=false;
        recentBlueCenter=false;
        recentBlueRight=false;

        recentRedLeft=false;
        recentRedCenter=false;
        recentRedRight=false;

        RED_BACKDROP_LEFT_TAG.isDetected =false;
        RED_BACKDROP_CENTER_TAG.isDetected =false;
        RED_BACKDROP_RIGHT_TAG.isDetected = false;

        BLUE_BACKDROP_LEFT_TAG.isDetected =false;
        BLUE_BACKDROP_CENTER_TAG.isDetected =false;
        BLUE_BACKDROP_RIGHT_TAG.isDetected = false;

        RED_BACKDROP_LEFT_TAG.timestamp = 0;
        RED_BACKDROP_CENTER_TAG.timestamp = 0;
        RED_BACKDROP_RIGHT_TAG.timestamp = 0;

        BLUE_BACKDROP_LEFT_TAG.timestamp = 0;
        BLUE_BACKDROP_CENTER_TAG.timestamp = 0;
        BLUE_BACKDROP_RIGHT_TAG.timestamp = 0;

        BLUE_BACKDROP_LEFT_TAG.detection=null;
        BLUE_BACKDROP_RIGHT_TAG.detection=null;
        BLUE_BACKDROP_CENTER_TAG.detection=null;

        RED_BACKDROP_LEFT_TAG.detection=null;
        RED_BACKDROP_RIGHT_TAG.detection=null;
        RED_BACKDROP_CENTER_TAG.detection=null;

        redBackdropAprilTagFoundRecently=false;
        blueBackdropAprilTagFoundRecently=false;
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
    public void DriverStationAprilTagTelemetry() {
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
//                    telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
//                    telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + "  (deg)");
//                    telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");

                } else {
                    telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                    telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
                }
            }
        }
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
                if (detection.ftcPose.range < 44)
                {
                    currentTag.setDetected();
                    currentTag.setTimestamp();
                    currentTag.storeDetection(detection);
                    double rangeError = (currentTag.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE_SAFETY);
                    // Pick whichever value is lower
                    double manualDriveLimit = Math.min(rangeError * tunableVisionConstants.SAFETY_SPEED_GAIN, tunableVisionConstants.MAX_MANUAL_BACKDROP_SPEED);
                    if (manualDriveLimit < DriveSubsystem.driveParameters.safetyDriveSpeedFactor) {
                        DriveSubsystem.driveParameters.safetyDriveSpeedFactor = manualDriveLimit;
                    }
                }
            }
        }

        blueBackdropAprilTagFoundRecently = CheckBlueBackdropAprilTags();
        redBackdropAprilTagFoundRecently = CheckRedBackdropAprilTags();
    }

    private boolean CheckBlueBackdropAprilTags() {
        recentBlueLeft = isTagSeenRecently(BLUE_BACKDROP_LEFT_TAG);
        recentBlueCenter = isTagSeenRecently(BLUE_BACKDROP_CENTER_TAG);
        recentBlueRight = isTagSeenRecently(BLUE_BACKDROP_RIGHT_TAG);
        if (    BLUE_BACKDROP_LEFT_TAG.isDetected || recentBlueLeft ||
                BLUE_BACKDROP_CENTER_TAG.isDetected || recentBlueCenter ||
                BLUE_BACKDROP_RIGHT_TAG.isDetected || recentBlueRight) {
            return true;
        } else
            return false;
    }

    private boolean CheckRedBackdropAprilTags() {
        recentRedLeft = isTagSeenRecently(RED_BACKDROP_LEFT_TAG);
        recentRedCenter = isTagSeenRecently(RED_BACKDROP_CENTER_TAG);
        recentRedRight = isTagSeenRecently(RED_BACKDROP_RIGHT_TAG);

        if (    RED_BACKDROP_LEFT_TAG.isDetected || recentRedLeft ||
                RED_BACKDROP_CENTER_TAG.isDetected || recentRedCenter ||
                RED_BACKDROP_RIGHT_TAG.isDetected || recentRedRight) {
            return true;
        } else return false;
    }

    private boolean isTagSeenRecently(AprilTagID tag){
        return (tag.getTimestamp() >
                MatchConfig.timestampTimer.seconds() - tunableVisionConstants.APRIL_TAG_LAST_SEEN_THRESHOLD_SEC);
    }

    private Pose2d DeterminePoseFromAprilTag(AprilTagID tag) {

        //get the field position of the AprilTag on the field and store it in a float vector (VectorF)
        VectorF tagVector = tag.detection.metadata.fieldPosition;

        //store the X value of the tag in tagPosXOnField access the individual values of the vector using .get(0)
        double tagPosXOnField = tagVector.get(0);

        //store the Y value of the tag - use .get(1) for this one
        double tagPosYOnField = tagVector.get(1);

        //the +90 rotates this tagheading to be facing the backdrop (it comes in at -90 for the backdrop)
        double tagHeading = QuaternionToHeading(tag.detection.metadata.fieldOrientation)+90;

        //Look at Fig. 2 at https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
        //save tag.detection.ftPose.x into a distanceY variable;
        //save tag.detection.ftcPose.y into a distanceX variable;
        //This is confusing because the ftCpose X and Y are not the X and Y coordinates of the field, but of the camera image
        //Ftcpose.x represents left and right on the camera image, which for the backdrop corresponds to the Y value.
        //Ftcpose.y represents distance to the robot, which for the backdrop is our X value

        double distanceY = tag.detection.ftcPose.x;  //this is correct even though it seems odd
        double distanceX = tag.detection.ftcPose.y;  //same

        //save the tag.detection.ftPose.yaw as the cameraYaw
        double cameraYaw = tag.detection.ftcPose.yaw;

        Pose2d newPose = new Pose2d(tagPosXOnField-distanceX, tagPosYOnField+distanceY, Robot.getInstance().getGyroSubsystem().currentRelativeYawRadians);

        telemetry.addLine();
        telemetry.addData("Tag", tag.detection.metadata.name);
//        telemetry.addData("Tag Pose", "X %5.2f, Y %5.2f, heading %5.2f ", tagPosXOnField, tagPosYOnField, tagHeading);
//        telemetry.addData("DistToCamera", "X %5.2f, , Y %5.2f, yaw %5.2f,", distanceX, distanceY, cameraYaw);

        telemetry.addData("New Pose", "X %5.2f, Y %5.2f, heading %5.2f ", newPose.position.x, newPose.position.y, Math.toDegrees(newPose.heading.log()));

        return newPose;
    }


    public double QuaternionToHeading (Quaternion quaternion) {

        // Calculate yaw (heading) from quaternion
        double t0 = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
        double t1 = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        double yawRadians = Math.atan2(t0, t1);

        // Convert yaw angle from radians to degrees
        double yawDegrees = Math.toDegrees(yawRadians);

        // Ensure the yaw angle is within the range [-180, 180] degrees
        if (yawDegrees > 180) {
            yawDegrees -= 360;
        } else if (yawDegrees < -180) {
            yawDegrees += 360;
        }

        return yawDegrees;
    }

    //  If right tag is detected AND one of the following three things is true, then drive to the right tag:
    //  1) the delivery location is right; or
    //  2) the delivery location is center, but center isn't found
    //  3) the delivery location is left, but left isn't found

    public boolean AutoDriveToBackdrop() {
        deliveryTag=null;
        offsetTagToLeft=null;
        offsetTagToRight=null;
        getDeliverLocationTags(getDeliverLocation());

        if (isTagSeenRecently(deliveryTag)) {
                CalculateErrors(deliveryTag);
                ClipErrorsAndSaveAprilTagDrivingValues();
                //these errors seem to be better for telling when we are finished.. (even though they aren't the same)
                return stillSeekingAprilTag(rangeError, xError, yawError, deliveryTag);
            }
            else if (offsetTagToLeft!=null && offsetTagToLeft.isDetected){
                CalculateErrors(deliveryTag);
                ClipErrorsAndSaveAprilTagDrivingValues();
                return true;
            }
            else if (offsetTagToRight!=null && offsetTagToRight.isDetected){
                CalculateErrors(deliveryTag);
                ClipErrorsAndSaveAprilTagDrivingValues();
                return true;
        }

        return false;
    }

    public boolean AutoDriveToBackdropBlue() {
        if  (
                (BLUE_BACKDROP_RIGHT_TAG.detection!=null &&
                        (BLUE_BACKDROP_RIGHT_TAG.isDetected || recentBlueRight))  &&
                        (
                            getDeliverLocation().equals(DeliverLocation.RIGHT) ||
                            (getDeliverLocation().equals(DeliverLocation.CENTER) && !BLUE_BACKDROP_CENTER_TAG.isDetected)  ||
                            (getDeliverLocation().equals(DeliverLocation.LEFT) && !BLUE_BACKDROP_LEFT_TAG.isDetected)
                        )
            )
        {
            CalculateErrors(BLUE_BACKDROP_RIGHT_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError,  BLUE_BACKDROP_RIGHT_TAG);
        }
        //Drive to the Center backdrop if its the deliver location, or
        // if its left, but left is not detected or
        // if its right, but right is not detected

        else if (
                (BLUE_BACKDROP_CENTER_TAG.detection!=null &&
                (BLUE_BACKDROP_CENTER_TAG.isDetected || recentBlueCenter))
                    && (
                    getDeliverLocation().equals(DeliverLocation.CENTER) ||
                    (getDeliverLocation().equals(DeliverLocation.LEFT) && !BLUE_BACKDROP_LEFT_TAG.isDetected) ||
                    (getDeliverLocation().equals(DeliverLocation.RIGHT) && !BLUE_BACKDROP_RIGHT_TAG.isDetected)))
        {
            CalculateErrors(BLUE_BACKDROP_CENTER_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError, BLUE_BACKDROP_CENTER_TAG);
        }

        //Drive to the Right backdrop if its the deliver location, or
        // if its left, but left is not detected or
        // if its center, but center is not detected
        else if (
                (BLUE_BACKDROP_LEFT_TAG.detection!=null &&
                (BLUE_BACKDROP_LEFT_TAG.isDetected || recentBlueLeft)) &&
                (getDeliverLocation().equals(DeliverLocation.LEFT)    ||
                (getDeliverLocation().equals(DeliverLocation.RIGHT) && !BLUE_BACKDROP_RIGHT_TAG.isDetected)       ||
                (getDeliverLocation().equals(DeliverLocation.CENTER) && !BLUE_BACKDROP_CENTER_TAG.isDetected)))
        {
            CalculateErrors(BLUE_BACKDROP_LEFT_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError, BLUE_BACKDROP_LEFT_TAG);
        }
        return false;
    }

    public boolean AutoDriveToBackdropRed() {
        if (
                (RED_BACKDROP_LEFT_TAG.detection!=null &&
                (RED_BACKDROP_LEFT_TAG.isDetected || recentRedLeft))
                && (getDeliverLocation().equals(DeliverLocation.LEFT)
                ||
                (getDeliverLocation().equals(DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)    ||
                (getDeliverLocation().equals(DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected)))
        {
            CalculateErrors(RED_BACKDROP_LEFT_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError, RED_BACKDROP_LEFT_TAG);

        } else if (
                (RED_BACKDROP_CENTER_TAG.detection!=null &&
                (RED_BACKDROP_CENTER_TAG.isDetected || recentRedCenter)) &&
                            (getDeliverLocation().equals(DeliverLocation.CENTER)
            ||              (getDeliverLocation().equals(DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                            (getDeliverLocation().equals(DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected))
                    )
        {
            CalculateErrors(RED_BACKDROP_CENTER_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError, RED_BACKDROP_CENTER_TAG);

        }
        else if (
                (RED_BACKDROP_RIGHT_TAG.detection!=null &&
                (RED_BACKDROP_RIGHT_TAG.isDetected || recentRedRight)) && (
                 getDeliverLocation().equals(DeliverLocation.RIGHT)
                         ||
                (getDeliverLocation().equals(DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                (getDeliverLocation().equals(DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)))
        {
            CalculateErrors(RED_BACKDROP_RIGHT_TAG);
            ClipErrorsAndSaveAprilTagDrivingValues();
            return stillSeekingAprilTag(rangeError, xError, yawError, RED_BACKDROP_RIGHT_TAG);
        }
        return false;
    }

    private void ClipErrorsAndSaveAprilTagDrivingValues() {
        double drive = ClipDrive(rangeError);
        double strafe = ClipStrafe(yawError);
        double turn = ClipTurn(bearingError);

        Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
        Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
        Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;
        telemetry.addData("Auto to Right Blue Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }

    private void CalculateErrors(AprilTagID tag) {
        MatchConfig.telemetryPacket.put(tag.toString(), tag.isDetected);
        rangeError = (tag.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
        xError = tag.detection.ftcPose.x;
        yawError = tag.detection.ftcPose.yaw;
        bearingError = tag.detection.ftcPose.bearing;
    }

    private double ClipDrive(double rangeError) {
        return Range.clip(
                    (rangeError * tunableVisionConstants.RANGE_GAIN) + //GAIN multiplied by Error
                        (Math.signum(rangeError)*tunableVisionConstants.RANGE_FEEDFORWARD), //add a feedforward in the correct direction
                                            -tunableVisionConstants.MAX_AUTO_SPEED, // clip to this low value
                                            tunableVisionConstants.MAX_AUTO_SPEED); // or this high value
    }
    private double ClipStrafe(double xError) {
          return Range.clip(
                  (xError * tunableVisionConstants.STRAFE_GAIN)
                          + (Math.signum(xError)*tunableVisionConstants.STRAFE_FEEDFORWARD),
                  -tunableVisionConstants.MAX_AUTO_STRAFE, tunableVisionConstants.MAX_AUTO_STRAFE);
    }

    private double ClipTurn(double bearingError) {
        return Range.clip(
                (-bearingError * tunableVisionConstants.TURN_GAIN)
                        + + (Math.signum(-bearingError)*tunableVisionConstants.TURN_FEEDFORWARD),
                -tunableVisionConstants.MAX_AUTO_TURN, tunableVisionConstants.MAX_AUTO_TURN);
    }

    public void setDeliverLocation(DeliverLocation location) {
        deliverLocation = location;
    }


    public DeliverLocation getDeliverLocation(){
        return deliverLocation;
    }

    //returns false once the pose reaches a steady state for a certain number of checks
    private boolean stillSeekingAprilTag(double rangeError, double yawError, double bearingError, AprilTagID tag) {
        //We have found the target if this is true
        if (    (Math.abs(rangeError)    < tunableVisionConstants.BACKDROP_DRIVE_THRESHOLD) &&
                (Math.abs(yawError)   < tunableVisionConstants.BACKDROP_STRAFE_THRESHOLD) &&
                (Math.abs(bearingError)     < tunableVisionConstants.BACKDROP_TURN_THRESHOLD)){

            backdropPoseCount++;
            if (backdropPoseCount > tunableVisionConstants.BACKDROP_POSE_COUNT_THRESHOLD){
                backdropPoseCount=0;
                resetPose = DeterminePoseFromAprilTag(tag);
                resetPoseReady = true;
                telemetry.addData("New Pose", "X %5.2f, Y %5.2f, heading %5.2f ", resetPose.position.x, resetPose.position.y, resetPose.heading.log());
                //We are at our target, stop running the AutoDrive code

                mecanumDrive.drive=0; mecanumDrive.strafe=0; mecanumDrive.turn=0;
                mecanumDrive.current_drive_ramp = 0; mecanumDrive.current_strafe_ramp=0; mecanumDrive.current_turn_ramp=0;
                mecanumDrive.aprilTagDrive=0; mecanumDrive.aprilTagStrafe=0; mecanumDrive.aprilTagTurn=0;

                return false;
            }
        }
        //Keep running the Autodrive method
        return true;
    }

    public boolean isDeliverLocationTagVisible(VisionSubsystem.DeliverLocation deliverLocation) {
        if (
                (deliverLocation==LEFT && RED_BACKDROP_LEFT_TAG.isDetected && MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.RED)
                        || (deliverLocation==CENTER && RED_BACKDROP_CENTER_TAG.isDetected && MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED)
                        || (deliverLocation==RIGHT && RED_BACKDROP_RIGHT_TAG.isDetected && MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.RED)
                        || (deliverLocation==LEFT && BLUE_BACKDROP_LEFT_TAG.isDetected && MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.BLUE)
                        || (deliverLocation==CENTER && BLUE_BACKDROP_CENTER_TAG.isDetected && MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE)
                        || (deliverLocation==RIGHT && BLUE_BACKDROP_RIGHT_TAG.isDetected && MatchConfig.finalAllianceColor== InitVisionProcessor.AllianceColor.BLUE))
        {
            return true;
        } else return false;
    }

    public void getDeliverLocationTags(VisionSubsystem.DeliverLocation deliverLocation) {
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
            if (deliverLocation == LEFT) {
                deliveryTag = RED_BACKDROP_LEFT_TAG;
                if (RED_BACKDROP_CENTER_TAG.isDetected) {
                    offsetTagToRight = RED_BACKDROP_CENTER_TAG;
                } else if (RED_BACKDROP_RIGHT_TAG.isDetected) {
                    offsetTagToRight = RED_BACKDROP_RIGHT_TAG;
                }
            }
            else if (deliverLocation == RIGHT) {
                deliveryTag = RED_BACKDROP_RIGHT_TAG;
                if (RED_BACKDROP_CENTER_TAG.isDetected) {
                    offsetTagToLeft = RED_BACKDROP_CENTER_TAG;
                } else if (RED_BACKDROP_LEFT_TAG.isDetected) {
                    offsetTagToLeft = RED_BACKDROP_LEFT_TAG;
                }
            }
            else {
                deliveryTag = RED_BACKDROP_CENTER_TAG;
                if (RED_BACKDROP_RIGHT_TAG.isDetected) {
                    offsetTagToRight = RED_BACKDROP_RIGHT_TAG;
                } else if (RED_BACKDROP_LEFT_TAG.isDetected) {
                    offsetTagToLeft = RED_BACKDROP_LEFT_TAG;
                }
            }
        } else {
            if (deliverLocation == LEFT) {
                deliveryTag = BLUE_BACKDROP_LEFT_TAG;
                if (BLUE_BACKDROP_CENTER_TAG.isDetected) {
                    offsetTagToRight = BLUE_BACKDROP_CENTER_TAG;
                } else if (BLUE_BACKDROP_RIGHT_TAG.isDetected) {
                    offsetTagToRight = BLUE_BACKDROP_RIGHT_TAG;
                }
            } else if (deliverLocation == RIGHT) {
                deliveryTag = BLUE_BACKDROP_RIGHT_TAG;
                if (BLUE_BACKDROP_CENTER_TAG.isDetected) {
                    offsetTagToLeft = BLUE_BACKDROP_CENTER_TAG;
                } else if (BLUE_BACKDROP_LEFT_TAG.isDetected) {
                    offsetTagToLeft = BLUE_BACKDROP_LEFT_TAG;
                }
            } else {
                deliveryTag = BLUE_BACKDROP_CENTER_TAG;
                if (BLUE_BACKDROP_RIGHT_TAG.isDetected) {
                    offsetTagToRight = BLUE_BACKDROP_RIGHT_TAG;
                } else if (BLUE_BACKDROP_LEFT_TAG.isDetected) {
                    offsetTagToLeft = BLUE_BACKDROP_LEFT_TAG;
                }
            }
        }
    }

}


