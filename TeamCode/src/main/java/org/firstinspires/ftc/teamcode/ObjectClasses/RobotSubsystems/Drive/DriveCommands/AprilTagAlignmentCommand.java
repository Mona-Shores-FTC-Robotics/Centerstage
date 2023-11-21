package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_DRIVING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_STRAFING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_TURNING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.MANUAL_DRIVE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.autoDriveParameters;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.BLUE_BACKDROP_CENTER_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.BLUE_BACKDROP_LEFT_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.BLUE_BACKDROP_RIGHT_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_CENTER_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_LEFT_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_RIGHT_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.CENTER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.tunableVisionConstants;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

/**
 * A command to drive the robot with joystick input *
 */
public class AprilTagAlignmentCommand extends CommandBase {

    private final VisionSubsystem visionSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final GyroSubsystem gyroSubsystem;

    private double drive;
    private double strafe;
    private double turn;

    private double rangeError;
    private double yawError;
    private double bearingError;
    private double xError;
    private double yError;

    private VisionSubsystem.DeliverLocation previousDeliverLocation;

    private TurnPIDController pidTurn;
    private PIDFController pidDrive;
    private PIDFController pidStrafe;

    /**
     * Creates a new DefaultDrive.
     */
    public AprilTagAlignmentCommand(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        gyroSubsystem = Robot.getInstance().getGyroSubsystem();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (visionSubsystem.isDeliverLocationTagVisible(visionSubsystem.getDeliverLocation()))
        {
            //set up the three PID controllers
            pidTurn = new TurnPIDController(0, autoDriveParameters.TURN_P, autoDriveParameters.TURN_I, autoDriveParameters.TURN_D, autoDriveParameters.TURN_F);
            pidStrafe = new PIDFController(autoDriveParameters.STRAFE_P, autoDriveParameters.STRAFE_I, autoDriveParameters.STRAFE_D, autoDriveParameters.STRAFE_F);
            pidStrafe.setSetPoint(0);
            pidDrive = new PIDFController(autoDriveParameters.DRIVE_P, autoDriveParameters.DRIVE_I, autoDriveParameters.DRIVE_D, autoDriveParameters.DRIVE_F);
            pidDrive.setSetPoint(0);
            previousDeliverLocation = visionSubsystem.getDeliverLocation();
            driveSubsystem.currentState=APRIL_TAG_ALIGNMENT_TURNING;
        } else driveSubsystem.currentState=MANUAL_DRIVE;
    }

    @Override
    public void execute() {
        pidTurn.setPIDF(autoDriveParameters.TURN_P, autoDriveParameters.TURN_I, autoDriveParameters.TURN_D, autoDriveParameters.TURN_F);
        pidStrafe.setPIDF(autoDriveParameters.STRAFE_P, autoDriveParameters.STRAFE_I, autoDriveParameters.STRAFE_D, autoDriveParameters.STRAFE_F);
        pidDrive.setPIDF(autoDriveParameters.DRIVE_P, autoDriveParameters.DRIVE_I, autoDriveParameters.DRIVE_D, autoDriveParameters.DRIVE_F);

        //This attempts to handle the situation where the driver changes the delivery location during the middle of the command
        CheckIfDeliveryLocationChanged();

        //this sets the drive/strafe/turn values for automatic alignment in four stages (TURNING, STRAFING TO SEE TAG, STRAFING TO ALIGN TO TAG, DRIVING TO TAG)
        setDriveStrafeTurnValuesForAprilTagAlignment();

        //this moves the robot using drive/strafe/turn values using speed control
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(drive, strafe, turn);

        //save the current delivery location as the previous delviery location in case the driver changes it mid-command;
        previousDeliverLocation = visionSubsystem.getDeliverLocation();
        MatchConfig.telemetryPacket.put("Deliver Location", previousDeliverLocation);
    }

    private void CheckIfDeliveryLocationChanged() {
        if (visionSubsystem.getDeliverLocation()!=previousDeliverLocation)
        {
            if (visionSubsystem.isDeliverLocationTagVisible(visionSubsystem.getDeliverLocation()))
            {
                initialize();
            } else driveSubsystem.currentState=MANUAL_DRIVE;
        }
    }

    @Override
    public boolean isFinished() {
        if (driveSubsystem.currentState==MANUAL_DRIVE) {
            return true;
        } else return false;
    }

    public void setDriveStrafeTurnValuesForAprilTagAlignment() {
        //Align to the Backdrop AprilTags - CASE RED
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED &&
                visionSubsystem.redBackdropAprilTagFoundRecently) {

            switch (driveSubsystem.currentState) {
                case APRIL_TAG_ALIGNMENT_TURNING: {
                    AutoDriveToBackdropRed();
                    drive = 0;
                    strafe = 0;
                    turn = pidTurn.update(gyroSubsystem.currentRelativeYawDegrees);

                    if (Math.abs(pidTurn.error) < autoDriveParameters.TURN_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG;
                    }
                    break;
                }


                case APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG: {
                    AutoDriveToBackdropRed();
                    drive = 0;
                    strafe = Math.signum(visionSubsystem.yawError) * autoDriveParameters.STRAFE_TO_TAG_SPEED;
                    turn = 0;

                    //strafe until we see the target again since we probably can't see it if we just turned
                    // this step essentially gets skipped if we can see the tag
                    if (visionSubsystem.isDeliverLocationTagVisible(visionSubsystem.getDeliverLocation()))
                    {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_STRAFING;
                    }
                    break;
                }

                case APRIL_TAG_ALIGNMENT_STRAFING: {
                    AutoDriveToBackdropRed();
                    drive = 0;
                    strafe = pidStrafe.calculate(visionSubsystem.yawError);
                    turn = 0;

                    if (Math.abs(Robot.getInstance().getVisionSubsystem().yawError) < autoDriveParameters.STRAFE_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_DRIVING;
                    }
                    break;
                }

                case APRIL_TAG_ALIGNMENT_DRIVING: {
                    AutoDriveToBackdropRed();
                    drive = pidDrive.calculate(visionSubsystem.rangeError);
                    strafe = 0;
                    turn = 0;

                    if (Math.abs(Robot.getInstance().getVisionSubsystem().rangeError) < autoDriveParameters.DRIVE_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = MANUAL_DRIVE;
                    }
                    break;
                }
            }
            MatchConfig.telemetryPacket.put("AprilTag Drive", JavaUtil.formatNumber(drive, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Range Error", JavaUtil.formatNumber(rangeError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag Strafe", JavaUtil.formatNumber(strafe, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Yaw Error", JavaUtil.formatNumber(bearingError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag Turn", JavaUtil.formatNumber(turn, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Heading Error", JavaUtil.formatNumber(yawError, 6, 6));
        }

        //Align to the Backdrop AprilTags - CASE BLUE
        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                visionSubsystem.blueBackdropAprilTagFoundRecently) {

            switch (driveSubsystem.currentState) {
                case APRIL_TAG_ALIGNMENT_TURNING: {
                    AutoDriveToBackdropBlue();
                    drive = 0;
                    strafe = 0;
                    turn = pidTurn.update(gyroSubsystem.currentRelativeYawDegrees);

                    if (Math.abs(pidTurn.error) < autoDriveParameters.TURN_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG;
                    }
                    break;
                }


                case APRIL_TAG_ALIGNMENT_STRAFING_TO_FIND_TAG: {
                    AutoDriveToBackdropBlue();
                    drive = 0;
                    strafe = Math.signum(visionSubsystem.yawError) * autoDriveParameters.STRAFE_TO_TAG_SPEED;
                    turn = 0;

                    //strafe until we see the target again since we probably can't see it if we just turned
                    // this step essentially gets skipped if we can see the tag
                    if (visionSubsystem.isDeliverLocationTagVisible(visionSubsystem.getDeliverLocation()))
                    {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_STRAFING;
                    }
                    break;
                }

                case APRIL_TAG_ALIGNMENT_STRAFING: {
                    AutoDriveToBackdropBlue();
                    drive = 0;
                    strafe = pidStrafe.calculate(visionSubsystem.yawError);
                    turn = 0;

                    if (Math.abs(Robot.getInstance().getVisionSubsystem().yawError) < autoDriveParameters.STRAFE_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_DRIVING;
                    }
                    break;
                }

                case APRIL_TAG_ALIGNMENT_DRIVING: {
                    AutoDriveToBackdropBlue();
                    drive = pidDrive.calculate(visionSubsystem.rangeError);
                    strafe = 0;
                    turn = 0;

                    if (Math.abs(Robot.getInstance().getVisionSubsystem().rangeError) < autoDriveParameters.DRIVE_ERROR_THRESHOLD) {
                        driveSubsystem.currentState = MANUAL_DRIVE;
                    }
                    break;
                }
            }
            MatchConfig.telemetryPacket.put("AprilTag Drive", JavaUtil.formatNumber(drive, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Range Error", JavaUtil.formatNumber(rangeError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag Strafe", JavaUtil.formatNumber(strafe, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Bearing Error", JavaUtil.formatNumber(bearingError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag Turn", JavaUtil.formatNumber(turn, 6, 6));
            MatchConfig.telemetryPacket.put("AprilTag Yaw Error", JavaUtil.formatNumber(yawError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag X Error", JavaUtil.formatNumber(xError, 6, 6));

            MatchConfig.telemetryPacket.put("AprilTag Y Error", JavaUtil.formatNumber(yError, 6, 6));
        }

    }

    public boolean AutoDriveToBackdropRed() {
        if ( RED_BACKDROP_LEFT_TAG.detection!=null &&
                (RED_BACKDROP_LEFT_TAG.isDetected || visionSubsystem.recentRedLeft) &&
                (visionSubsystem.getDeliverLocation().equals(LEFT)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            RED_BACKDROP_LEFT_TAG.rangeError =  rangeError = (RED_BACKDROP_LEFT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            RED_BACKDROP_LEFT_TAG.yawError = yawError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.yaw;
            RED_BACKDROP_LEFT_TAG.bearingError = bearingError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.bearing;
            RED_BACKDROP_LEFT_TAG.xError = xError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.x;
            RED_BACKDROP_LEFT_TAG.yError = yError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Red Left Tag", RED_BACKDROP_LEFT_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Red Left Seen Recently", visionSubsystem.recentRedLeft);
            return atBackDrop(rangeError, yawError, bearingError, RED_BACKDROP_LEFT_TAG);

        } else if ( RED_BACKDROP_CENTER_TAG.detection!=null &&
                        (RED_BACKDROP_CENTER_TAG.isDetected || visionSubsystem.recentRedCenter) &&
                        (visionSubsystem.getDeliverLocation().equals(CENTER)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            RED_BACKDROP_CENTER_TAG.rangeError = rangeError = (RED_BACKDROP_CENTER_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            RED_BACKDROP_CENTER_TAG.yawError = yawError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.yaw;
            RED_BACKDROP_CENTER_TAG.bearingError = bearingError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.bearing;
            RED_BACKDROP_CENTER_TAG.xError = xError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.x;
            RED_BACKDROP_CENTER_TAG.yError = yError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Red Center Tag", RED_BACKDROP_CENTER_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Red Center Seen Recently", visionSubsystem.recentRedCenter);

            return atBackDrop(rangeError, yawError, bearingError, RED_BACKDROP_CENTER_TAG);
        } else if ( RED_BACKDROP_RIGHT_TAG.detection!=null &&
                    (RED_BACKDROP_RIGHT_TAG.isDetected || visionSubsystem.recentRedRight) &&
                    visionSubsystem.getDeliverLocation().equals(RIGHT))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            RED_BACKDROP_RIGHT_TAG.rangeError =  rangeError = (RED_BACKDROP_RIGHT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            RED_BACKDROP_RIGHT_TAG.yawError = yawError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.yaw;
            RED_BACKDROP_RIGHT_TAG.bearingError = bearingError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.bearing;
            RED_BACKDROP_RIGHT_TAG.xError = xError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.x;
            RED_BACKDROP_RIGHT_TAG.yError = yError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Red Right Tag", RED_BACKDROP_RIGHT_TAG);
            MatchConfig.telemetryPacket.put("Red Right Seen Recently", visionSubsystem.recentRedRight);

            return atBackDrop(rangeError, yawError, bearingError, RED_BACKDROP_RIGHT_TAG);
        }
        return false;
    }

    public boolean AutoDriveToBackdropBlue() {
        if ( BLUE_BACKDROP_LEFT_TAG.detection!=null &&
                (BLUE_BACKDROP_LEFT_TAG.isDetected || visionSubsystem.recentBlueLeft) &&
                (visionSubsystem.getDeliverLocation().equals(LEFT)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            BLUE_BACKDROP_LEFT_TAG.rangeError = rangeError = (BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            BLUE_BACKDROP_LEFT_TAG.yawError = yawError = BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.yaw;
            BLUE_BACKDROP_LEFT_TAG.bearingError = bearingError = BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.bearing;
            BLUE_BACKDROP_LEFT_TAG.xError = xError = BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.x;
            BLUE_BACKDROP_LEFT_TAG.yError = yError =  BLUE_BACKDROP_LEFT_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Blue Left Tag", BLUE_BACKDROP_LEFT_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Blue Left Seen Recently", visionSubsystem.recentBlueLeft);
            return atBackDrop(rangeError, yawError, bearingError, BLUE_BACKDROP_LEFT_TAG);

        } else if ( BLUE_BACKDROP_CENTER_TAG.detection!=null &&
                (BLUE_BACKDROP_CENTER_TAG.isDetected || visionSubsystem.recentBlueCenter) &&
                (visionSubsystem.getDeliverLocation().equals(CENTER)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            BLUE_BACKDROP_CENTER_TAG.rangeError =  rangeError = (BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            BLUE_BACKDROP_CENTER_TAG.yawError = yawError = BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.yaw;
            BLUE_BACKDROP_CENTER_TAG.bearingError =  bearingError = BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.bearing;
            BLUE_BACKDROP_CENTER_TAG.xError = xError = BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.x;
            BLUE_BACKDROP_CENTER_TAG.yError = yError =  BLUE_BACKDROP_CENTER_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Blue Center Tag", BLUE_BACKDROP_CENTER_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Blue Center Seen Recently", visionSubsystem.recentBlueCenter);

            return atBackDrop(rangeError, yawError, bearingError, BLUE_BACKDROP_CENTER_TAG);
        } else if ( BLUE_BACKDROP_RIGHT_TAG.detection!=null &&
                (BLUE_BACKDROP_RIGHT_TAG.isDetected || visionSubsystem.recentBlueRight) &&
                visionSubsystem.getDeliverLocation().equals(RIGHT))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            BLUE_BACKDROP_RIGHT_TAG.rangeError = rangeError = (BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            BLUE_BACKDROP_RIGHT_TAG.yawError = yawError = BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.yaw;
            BLUE_BACKDROP_RIGHT_TAG.bearingError = bearingError = BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.bearing;
            BLUE_BACKDROP_RIGHT_TAG.xError = xError = BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.x;
            BLUE_BACKDROP_RIGHT_TAG.yError = yError =  BLUE_BACKDROP_RIGHT_TAG.detection.ftcPose.y;

            MatchConfig.telemetryPacket.put("Blue Right Tag", BLUE_BACKDROP_RIGHT_TAG);
            MatchConfig.telemetryPacket.put("Blue Right Seen Recently", visionSubsystem.recentBlueRight);

            return atBackDrop(rangeError, yawError, bearingError, BLUE_BACKDROP_RIGHT_TAG);
        }
        return false;
    }

    //returns false once the pose reaches a steady state for a certain number of checks
    private boolean atBackDrop(double rangeError, double yawError, double bearingError, VisionSubsystem.AprilTagID tag) {
        //We have found the target if this is true
        if (    (Math.abs(rangeError)    < tunableVisionConstants.BACKDROP_DRIVE_THRESHOLD) &&
                (Math.abs(yawError)   < tunableVisionConstants.BACKDROP_TURN_THRESHOLD) &&
                (Math.abs(bearingError)     < tunableVisionConstants.BACKDROP_STRAFE_THRESHOLD)){

            visionSubsystem.backdropPoseCount++;
            if (visionSubsystem.backdropPoseCount > tunableVisionConstants.BACKDROP_POSE_COUNT_THRESHOLD){
                visionSubsystem.backdropPoseCount=0;
                visionSubsystem.resetPose = DeterminePoseFromAprilTag(tag);
                visionSubsystem.resetPoseReady = true;
                MatchConfig.telemetryPacket.put("Reset Pose X", visionSubsystem.resetPose.position.x);
                MatchConfig.telemetryPacket.put("Reset Pose Y", visionSubsystem.resetPose.position.y);
                MatchConfig.telemetryPacket.put("Reset Pose Heading", visionSubsystem.resetPose.heading.log());
                return true;
            }
        }
        return false;
    }

    private Pose2d DeterminePoseFromAprilTag(VisionSubsystem.AprilTagID tag) {

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

}
