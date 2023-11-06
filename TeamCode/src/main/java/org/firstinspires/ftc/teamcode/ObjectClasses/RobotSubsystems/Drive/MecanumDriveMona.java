package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.FollowTrajectoryAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.TurnAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;

import org.firstinspires.ftc.teamcode.Roadrunner.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;

@Config
public final class MecanumDriveMona {

    public static class ParamsMona {
        public double DRIVE_RAMP = .06; //ken ramp
        public double STRAFE_RAMP = .05;
        public double TURN_RAMP = .05;

        public double RAMP_THRESHOLD = .04; // This is the threshold at which we just clamp to the target drive/strafe/turn value

        //it looks to me like just using a feedforward of 12.5 gets the actual speed to match the target. The PID doesn't seem to really do anything.
        public double P =0; // default = 10
        public double D =0; // default = 0
        public double I =0; // default = 3
        public double F =8; // default = 0
    }

    public static class ParamsRRMona {
        /** Set Roadrunner motor parameters for faster drive motors **/

        // drive model parameters
        public double inPerTick =0.0317919075144509; //60.5\1903
        public double lateralInPerTick =0.0325115144947169; // 60\1845.5
        public double trackWidthTicks =631.8289216104534;

        // feedforward parameters (in tick units)
        public double kS =0.9574546275336608;
        public double kV =0.004264232249424524;
        public double kA =0.00055;

        // path profile parameters (in inches)
        public double maxWheelVel =25;
        public double minProfileAccel =-30;
        public double maxProfileAccel =30;

        // turn profile parameters (in radians)
        public double maxAngVel =Math.PI; // shared with path
        public double maxAngAccel =Math.PI;

        // path controller gains
        public double axialGain =12;
        public double lateralGain =8;
        public double headingGain =8; // shared with turn

        public double axialVelGain =1.1;
        public double lateralVelGain =1.1;
        public double headingVelGain =1.1; // shared with turn
    }

    public static class ParamsDriveTrainConstants {
        // DriveTrain physical constants
        public static double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        public static double TICKS_PER_REV = 384.5;
        public static double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;
    }

    public static ParamsMona MotorParameters = new ParamsMona();
    public static ParamsRRMona MotorParametersRR = new ParamsRRMona();
    public static ParamsDriveTrainConstants DriveTrainConstants = new ParamsDriveTrainConstants();

    public double drive, strafe, turn;
    public double last_drive=0, last_strafe=0, last_turn=0;
    public double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;
    public double aprilTagDrive, aprilTagStrafe, aprilTagTurn;
    private double leftFrontTargetSpeed, rightFrontTargetSpeed, leftBackTargetSpeed, rightBackTargetSpeed;

    public MecanumKinematics kinematics;
    public MotorFeedforward feedforward;
    public TurnConstraints defaultTurnConstraints;
    public VelConstraint defaultVelConstraint;
    public AccelConstraint defaultAccelConstraint;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public VoltageSensor voltageSensor;
    public Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public MecanumDriveMona() {
    }

    public void init() {
        HardwareMap hardwareMap = Robot.getInstance().getActiveOpMode().hardwareMap;
        this.pose = new Pose2d(0, 0, 0);

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "LFDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "LBDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "RBDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFDrive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer();

        //set the PID values one time
        leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);

        //Initialize the Roadrunner parameters (kinematics, feedforward, etc.)
        SetRoadRunnerParameters();
    }

    private void SetRoadRunnerParameters() {
        kinematics = new MecanumKinematics(
                MotorParametersRR.inPerTick * MotorParametersRR.trackWidthTicks, MotorParametersRR.inPerTick / MotorParametersRR.lateralInPerTick);

        feedforward = new MotorFeedforward(MotorParametersRR.kS, MotorParametersRR.kV / MotorParametersRR.inPerTick, MotorParametersRR.kA / MotorParametersRR.inPerTick);

        defaultTurnConstraints = new TurnConstraints(
                MotorParametersRR.maxAngVel, -MotorParametersRR.maxAngAccel, MotorParametersRR.maxAngAccel);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(MotorParametersRR.maxWheelVel),
                        new AngularVelConstraint(MotorParametersRR.maxAngVel)
                ));

        defaultAccelConstraint = new ProfileAccelConstraint(MotorParametersRR.minProfileAccel, MotorParametersRR.maxProfileAccel);

        FlightRecorder.write("MECANUM_RR_PARAMS", MotorParametersRR);
        FlightRecorder.write("MECANUM_PID_PARAMS", MotorParameters);
    }


    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {
        if (drive==0 && strafe ==0 && turn==0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            leftFront.setVelocity(0);
            leftBack.setVelocity(0);
            rightFront.setVelocity(0);
            rightBack.setVelocity(0);

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

        } else
        {

            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            //safetydrivespeedfactor is set when we lookforapriltags based on the closest backdrop apriltag we see (for the oposite alliance color)
            if (Robot.getInstance().getVisionSubsystem().blueBackdropAprilTagFound &&
                    Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.RED &&
                    drive > .1) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }
            //If we see red tags and we are blue and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            else if (Robot.getInstance().getVisionSubsystem().redBackdropAprilTagFound &&
                    Robot.getInstance().getVisionSubsystem().getInitVisionProcessor().allianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    drive > .1) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }


            //todo cleaned this up, it needs to be tested
            current_drive_ramp = Ramp(drive, current_drive_ramp, MotorParameters.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, MotorParameters.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, MotorParameters.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            leftFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            leftFront.setVelocity(leftFrontTargetSpeed);
            rightFront.setVelocity(rightFrontTargetSpeed);
            leftBack.setVelocity(leftBackTargetSpeed);
            rightBack.setVelocity(rightBackTargetSpeed);

            last_drive=drive;
            last_strafe=strafe;
            last_turn=turn;
        }
    }

    private double Ramp(double target, double currentValue, double ramp_amount) {
        if (Math.abs(currentValue) + MotorParameters.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }



    // This is what updates the pose estimate of the robot using the localizer
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        return twist.velocity().value();
    }

    public void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }

    public void telemetryDriveTrain() {

        Robot.getInstance().getActiveOpMode().telemetry.addLine("");

        Robot.getInstance().getActiveOpMode().telemetry.addData("Drive: ", drive);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Strafe: ", strafe);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Turn: ", turn);

        double targetSpeedLF = Math.round(100.0 * leftFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedRF = Math.round(100.0 * rightFrontTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedLB = Math.round(100.0 * leftBackTargetSpeed / DriveTrainConstants.TICKS_PER_REV);
        double targetSpeedRB = Math.round(100.0 * rightBackTargetSpeed / DriveTrainConstants.TICKS_PER_REV);

        double actualSpeedLF = Math.round(100.0 * leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedRF = Math.round(100.0 * rightFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedLB = Math.round(100.0 * leftBack.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double actualSpeedRB = Math.round(100.0 * rightBack.getVelocity() / DriveTrainConstants.TICKS_PER_REV);

        Robot.getInstance().getActiveOpMode().telemetry.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * leftFront.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("RF" + " Speed: " + JavaUtil.formatNumber(actualSpeedRF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRF, 4, 1) + " " + "Power: " + Math.round(100.0 * rightFront.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("LB" + " Speed: " + JavaUtil.formatNumber(actualSpeedLB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLB, 4, 1) + " " + "Power: " + Math.round(100.0 * leftBack.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("RB" + " Speed: " + JavaUtil.formatNumber(actualSpeedRB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRB, 4, 1) + " " + "Power: " + Math.round(100.0 * rightBack.getPower()) / 100.0);

        //todo make sure this doesn't mess up the field overlay in the dashboard (e.g., because this packet doesn't send that data)
        TelemetryPacket p = new TelemetryPacket();

        p.put("actualSpeedLF", actualSpeedLF);
        p.put("actualSpeedRF", actualSpeedRF);
        p.put("actualSpeedLB", actualSpeedLB);
        p.put("actualSpeedRB", actualSpeedRB);
        p.put("targetSpeedLF", targetSpeedLF);
        p.put("targetSpeedRF", targetSpeedRF);
        p.put("targetSpeedLB", targetSpeedLB);
        p.put("targetSpeedRB", targetSpeedRB);

        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    public void setMotorPower (double lF, double rF, double lB, double rB){
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        leftBack.setPower(lB);
        rightBack.setPower(rB);
    }


    public void mecanumDrivePowerControl (){
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        double leftFrontPower = ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        double rightFrontPower = ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        double leftBackPower = ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        double rightBackPower = ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }



}


