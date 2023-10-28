package org.firstinspires.ftc.teamcode.ObjectClasses.Commands.DriveCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    private double controllerDrive;
    private double controllerStrafe;
    private double controllerTurn;

    private double drive;
    private double strafe;
    private double turn;

    private boolean drivingToAprilTag;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command will run on.

     */
    public DefaultDrive(DriveSubsystem subsystem,
                        DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        drivingToAprilTag=false;
    }

    @Override
    public void execute() {
            setDriveStrafeTurnValues();
            driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(drive, strafe, turn);
    }

    public void setDriveStrafeTurnValues(){

        controllerDrive= driveSupplier.getAsDouble();
        controllerStrafe= strafeSupplier.getAsDouble();
        controllerTurn=turnSupplier.getAsDouble();

        //Check if driver controls are active so we can cancel automated driving if they are
        if (GamepadHandling.driverGamepadIsActive(controllerDrive, controllerStrafe, controllerTurn) || drivingToAprilTag) {

            //Increase strafe dead zone to 30%
            if (Math.abs(controllerStrafe) < .3) {controllerStrafe=0;}

            //apply speed factors
            controllerDrive = controllerDrive * driveSubsystem.mecanumDrive.MotorParameters.DRIVE_SPEED_FACTOR;
            controllerStrafe = controllerStrafe * driveSubsystem.mecanumDrive.MotorParameters.STRAFE_SPEED_FACTOR;
            controllerTurn = controllerTurn * driveSubsystem.mecanumDrive.MotorParameters.TURN_SPEED_FACTOR;

            // Cancel AprilTag driving if the driver is moving away from the backdrop
            if (controllerDrive < -.1) drivingToAprilTag = false;

            //Align to the Backdrop AprilTags - CASE RED
            if (visionSubsystem.getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    visionSubsystem.redBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropRed();
                controllerDrive = driveSubsystem.mecanumDrive.aprilTagDrive;
                controllerStrafe = driveSubsystem.mecanumDrive.aprilTagStrafe;
                controllerTurn = driveSubsystem.mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            }

            //Aligning to the Backdrop AprilTags - CASE BLUE
            else if (visionSubsystem.getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    visionSubsystem.blueBackdropAprilTagFound &&
                    (controllerDrive > .1 || drivingToAprilTag) &&
                    !GamepadHandling.getOverrideAprilTagDriving()) {
                visionSubsystem.AutoDriveToBackdropBlue();
                controllerDrive = driveSubsystem.mecanumDrive.aprilTagDrive;
                controllerStrafe = driveSubsystem.mecanumDrive.aprilTagStrafe;
                controllerTurn = driveSubsystem.mecanumDrive.aprilTagTurn;
                drivingToAprilTag = true;
            } else drivingToAprilTag = false;
        } else {
            // if we aren't automated driving and the sticks aren't out of the deadzone set it all to zero to stop us from moving
            controllerDrive = 0;
            controllerStrafe = 0;
            controllerTurn = 0;
        }
        drive = controllerDrive;
        strafe = controllerStrafe;
        turn = controllerTurn;
    }
}