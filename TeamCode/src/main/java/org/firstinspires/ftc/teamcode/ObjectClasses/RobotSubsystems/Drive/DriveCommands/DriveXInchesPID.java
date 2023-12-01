package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR19429;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

@Config
public class DriveXInchesPID extends CommandBase {
    // PIDF Constants
    public static double KP = 0.1; // Proportional coefficient
    public static double KI = 0.0; // Integral coefficient
    public static double KD = 0.0; // Derivative coefficient
    public static double KF = 0.0; // Feed-forward coefficient
    public static double PID_TOLERANCE = .5; // Tolerance for PID controller

    private final DriveSubsystem driveSubsystem;
    private final double targetDistanceInches;
    private PIDFController pidController;
    private double initialEncoderCount;
    private double targetEncoderCount;

    public DriveXInchesPID(DriveSubsystem subsystem, double inches) {
        this.driveSubsystem = subsystem;
        this.targetDistanceInches = inches;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetEncoders();
        initialEncoderCount = driveSubsystem.getCurrentEncoderCount();
        double targetDistanceEncoderCounts = convertInchesToEncoderCounts(targetDistanceInches);
        targetEncoderCount = initialEncoderCount + targetDistanceEncoderCounts;

        // Initialize the PID controller with constants
        pidController = new PIDFController(KP, KI, KD, KF);
        pidController.setSetPoint(targetEncoderCount);
        pidController.setTolerance(PID_TOLERANCE);
        pidController.reset();
    }

    @Override
    public void execute() {
        pidController.setPIDF(KP, KI, KD, KF); // Update PID coefficients if they are changed

        double currentEncoderCount = driveSubsystem.getCurrentEncoderCount();
        double output = pidController.calculate(currentEncoderCount);
        double clippedOutput = Range.clip(output, -.3, .3);
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(clippedOutput, 0, 0); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(0, 0, 0); // Stop the robot
    }

    private double convertInchesToEncoderCounts(double inches) {
        // Conversion logic here based on your robot's specific configuration
        // Example: return inches * ENCODER_COUNTS_PER_INCH;
        return inches * MotorParametersRR19429.inPerTick;
    }
}
