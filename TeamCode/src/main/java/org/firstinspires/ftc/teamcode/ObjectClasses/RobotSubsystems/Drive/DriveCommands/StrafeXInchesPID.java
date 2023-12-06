package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

@Config
public class StrafeXInchesPID extends CommandBase {
    // PIDF Constants
    public static double KP = -0.286; // Proportional coefficient
    public static double KI = 0.0; // Integral coefficient
    public static double KD = 0.0; // Derivative coefficient
    public static double KF = 0.0; // Feed-forward coefficient
    public static double PID_TOLERANCE = .001;

    private final DriveSubsystem driveSubsystem;
    private final double targetDistanceInches;
    private PIDFController pidController;
    private double initialEncoderCount;
    private double targetEncoderCount;

    public StrafeXInchesPID(DriveSubsystem subsystem, double inches) {
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
        pidController.reset();
        pidController.setSetPoint(targetEncoderCount);
        MatchConfig.telemetryPacket.put("StrafeXInchesPID target encoder", targetEncoderCount);

        pidController.setTolerance(PID_TOLERANCE);
    }

    @Override
    public void execute() {
        //This is done so we can change the values live with @Config
        pidController.setPIDF(KP,KI,KD,KF);

        //Get the current encoder count
        double currentEncoderCount = driveSubsystem.getCurrentEncoderCount();

        MatchConfig.telemetryPacket.put("StrafeXInchesPID current encoder", currentEncoderCount);

        //Calculate the velocity we need using the PID Controller
        double velocity = pidController.calculate(currentEncoderCount);
        double clippedVelocity = Range.clip(velocity, -.4, .4);
        MatchConfig.telemetryPacket.put("StrafeXInchesPID velocity", clippedVelocity);

        //Strafe the velocity provided by the PID
        driveSubsystem.mecanumDriveSpeedControl(0, clippedVelocity, 0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.mecanumDriveSpeedControl(0, 0, 0);
    }

    private double convertInchesToEncoderCounts(double inches) {
        return inches * PARAMS.lateralInPerTick;
    }
}