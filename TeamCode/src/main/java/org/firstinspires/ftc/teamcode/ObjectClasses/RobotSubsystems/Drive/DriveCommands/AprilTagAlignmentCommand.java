package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_START;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveStates.APRIL_TAG_ALIGNMENT_TURNING;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_CENTER_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_LEFT_TAG;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.AprilTagID.RED_BACKDROP_RIGHT_TAG;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.GyroSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

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
        //set up the PID controllers

        pidTurn = new TurnPIDController(0, autoDriveParameters.TURN_P, autoDriveParameters.TURN_I, autoDriveParameters.TURN_D, autoDriveParameters.TURN_F);
        pidStrafe = new PIDFController(autoDriveParameters.STRAFE_P, autoDriveParameters.STRAFE_I, autoDriveParameters.STRAFE_D, autoDriveParameters.STRAFE_F);
        pidStrafe.setSetPoint(0);
        pidDrive = new PIDFController(autoDriveParameters.DRIVE_P, autoDriveParameters.DRIVE_I, autoDriveParameters.DRIVE_D, autoDriveParameters.DRIVE_F);
        pidDrive.setSetPoint(0);

        driveSubsystem.currentState = APRIL_TAG_ALIGNMENT_TURNING;


    }

    @Override
    public void execute() {
        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        setDriveStrafeTurnValuesForAprilTagAlignment();
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }


    public void setDriveStrafeTurnValuesForAprilTagAlignment(){

        //Check if driver controls are active so we can cancel automated driving if they are


            //Align to the Backdrop AprilTags - CASE RED
            if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED &&
                    visionSubsystem.redBackdropAprilTagFoundRecently){
                if (driveSubsystem.currentState==APRIL_TAG_ALIGNMENT_START) {




                }

                visionSubsystem.AutoDriveToBackdropRed();
                drive = 0;
                strafe = 0;
                turn = pid.update(gyroSubsystem.currentRelativeYawDegrees);
                MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(turn, 6, 6));
            }

                                  if (!aprilTagAutoDriving)
                        {
                            aprilTagAutoDriving =true;
                            aprilTagTurning =false;
                            aprilTagStrafing=true;
                            aprilTagDriving=false;

                        }

                        if (aprilTagTurning){

                            visionSubsystem.AutoDriveToBackdropRed();
                            leftYAdjusted = 0;
                            leftXAdjusted = 0;
                            rightXAdjusted = pidTurn.update(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees);

                            if (Math.abs(pidTurn.error) < autoDriveParameters.TURN_ERROR_THRESHOLD)
                            {
                                aprilTagTurning =false;
                                aprilTagStrafing=false;
                                aprilTagDriving=true;
                            }
                        }

                        if (aprilTagStrafing){
                            visionSubsystem.AutoDriveToBackdropRed();
                            leftYAdjusted = 0;
                            leftXAdjusted = pidStrafe.calculate(visionSubsystem.yawError);
                            rightXAdjusted = 0;

                            MatchConfig.telemetryPacket.put("pidStrafeOutput", leftXAdjusted);

                            if (Math.abs(Robot.getInstance().getVisionSubsystem().yawError) < autoDriveParameters.STRAFE_ERROR_THRESHOLD)
                            {
                                aprilTagTurning =true;
                                aprilTagStrafing=false;
                                aprilTagDriving=false;
                            }
                        }

                        if (aprilTagDriving) {
                            visionSubsystem.AutoDriveToBackdropRed();
                            leftYAdjusted = pidDrive.calculate(visionSubsystem.rangeError);
                            leftXAdjusted = 0;
                            rightXAdjusted = 0;
                            if (Math.abs(Robot.getInstance().getVisionSubsystem().rangeError) < autoDriveParameters.DRIVE_ERROR_THRESHOLD)
                            {
                                aprilTagTurning =false;
                                aprilTagStrafing=false;
                                aprilTagDriving=false;
                                aprilTagAutoDriving=false;
                            }
                        }
                        MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(mecanumDrive.aprilTagDrive, 6, 6));
                        MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(mecanumDrive.aprilTagStrafe, 6, 6));
                        MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(mecanumDrive.aprilTagTurn, 6, 6));
                    }
                    //Aligning to the Backdrop AprilTags - CASE BLUE
                    else if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                            visionSubsystem.blueBackdropAprilTagFoundRecently &&
                            (leftYAdjusted > .2 || aprilTagAutoDriving) &&
                            !getOverrideAprilTagDriving()) {
                        aprilTagAutoDriving = visionSubsystem.AutoDriveToBackdropBlue();
                        leftYAdjusted = mecanumDrive.aprilTagDrive;
                        leftXAdjusted = mecanumDrive.aprilTagStrafe;
                        rightXAdjusted = mecanumDrive.aprilTagTurn;
                        MatchConfig.telemetryPacket.put("April Tag Drive", JavaUtil.formatNumber(mecanumDrive.aprilTagDrive, 6, 6));
                        MatchConfig.telemetryPacket.put("April Tag Strafe", JavaUtil.formatNumber(mecanumDrive.aprilTagStrafe, 6, 6));
                        MatchConfig.telemetryPacket.put("April Tag Turn", JavaUtil.formatNumber(mecanumDrive.aprilTagTurn, 6, 6));
                    } else aprilTagAutoDriving = false;
                } else {
                    // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
                    leftYAdjusted = 0;
                    leftXAdjusted = 0;
                    rightXAdjusted = 0;
                }
                drive = leftYAdjusted;
                strafe = leftXAdjusted;
                turn = rightXAdjusted;
            }


    public boolean AutoDriveToBackdropRed() {
        if (
                (RED_BACKDROP_LEFT_TAG.detection!=null &&
                        (RED_BACKDROP_LEFT_TAG.isDetected || recentRedLeft))
                        && (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.LEFT)
                        ||
                        (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)    ||
                        (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected)))
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError = (RED_BACKDROP_LEFT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            headingError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.bearing;
            yawError = RED_BACKDROP_LEFT_TAG.detection.ftcPose.yaw;

            double drive = ClipDrive(rangeError);
            double turn = ClipTurn(headingError);
            double strafe = ClipStrafe(yawError);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            MatchConfig.telemetryPacket.put("Red Left Tag", RED_BACKDROP_LEFT_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Red Left Seen Recently", recentRedLeft);
            MatchConfig.telemetryPacket.put("AprilTag Range Error", rangeError);
            MatchConfig.telemetryPacket.put("AprilTag Yaw Error", yawError);
            MatchConfig.telemetryPacket.put("AprilTag Heading Error", headingError);

            return stillSeekingAprilTag(rangeError, headingError, yawError, RED_BACKDROP_LEFT_TAG);

        } else if (
                (RED_BACKDROP_CENTER_TAG.detection!=null &&
                        (RED_BACKDROP_CENTER_TAG.isDetected || recentRedCenter)) &&
                        (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.CENTER)
                                ||              (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                                (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.RIGHT) && !RED_BACKDROP_RIGHT_TAG.isDetected))
        )
        {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError = (RED_BACKDROP_CENTER_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            headingError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.bearing;
            yawError = RED_BACKDROP_CENTER_TAG.detection.ftcPose.yaw;

            double drive = ClipDrive(rangeError);
            double turn = ClipTurn(headingError);
            double strafe = ClipStrafe(yawError);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            MatchConfig.telemetryPacket.put("Red Center Tag", RED_BACKDROP_CENTER_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Red Center Seen Recently", recentRedCenter);
            MatchConfig.telemetryPacket.put("AprilTag Range Error", rangeError);
            MatchConfig.telemetryPacket.put("AprilTag Yaw Error", yawError);
            MatchConfig.telemetryPacket.put("AprilTag Heading Error", headingError);

            telemetry.addData("Auto to Center Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            return stillSeekingAprilTag(rangeError, headingError, yawError, RED_BACKDROP_CENTER_TAG);

        }
        else if (
                (RED_BACKDROP_RIGHT_TAG.detection!=null &&
                        (RED_BACKDROP_RIGHT_TAG.isDetected || recentRedRight)) && (
                        getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.RIGHT)
                                ||
                                (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.LEFT) && !RED_BACKDROP_LEFT_TAG.isDetected)    ||
                                (getDeliverLocationRed().equals(VisionSubsystem.DeliverLocation.CENTER) && !RED_BACKDROP_CENTER_TAG.isDetected)))
        {

            MatchConfig.telemetryPacket.put("Red Right Tag", RED_BACKDROP_RIGHT_TAG);
            MatchConfig.telemetryPacket.put("Red Right Seen Recently", recentRedRight);
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError = (RED_BACKDROP_RIGHT_TAG.detection.ftcPose.range - tunableVisionConstants.DESIRED_DISTANCE);
            headingError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.bearing;
            yawError = RED_BACKDROP_RIGHT_TAG.detection.ftcPose.yaw;

            double drive = ClipDrive(rangeError);
            double turn = ClipTurn(headingError);
            double strafe = ClipStrafe(yawError);

            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagDrive = drive;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagStrafe = strafe;
            Robot.getInstance().getDriveSubsystem().mecanumDrive.aprilTagTurn = turn;

            MatchConfig.telemetryPacket.put("Red Center Tag", RED_BACKDROP_RIGHT_TAG.isDetected);
            MatchConfig.telemetryPacket.put("Red Center Seen Recently", recentRedRight);
            MatchConfig.telemetryPacket.put("AprilTag Range Error", rangeError);
            MatchConfig.telemetryPacket.put("AprilTag Yaw Error", yawError);
            MatchConfig.telemetryPacket.put("AprilTag Heading Error", headingError);

            telemetry.addData("Auto to Right Red Backdrop", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            return stillSeekingAprilTag(rangeError, headingError, yawError, RED_BACKDROP_RIGHT_TAG);
        }
        return false;
    }

}