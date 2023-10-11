package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.RobotType.ROBOT_VISION_FAST_MOTORS;
import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.Constants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.MotorParameters.Params;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDriveMona {

    public double drive;
    public double strafe;
    public double turn;

    private double DEFAULT_P = 11; // default = 10
    private double DEFAULT_D = 3; // default = 0
    private double DEFAULT_I = 0; // default = 3
    private double DEFAULT_F = 12; // default = 0

    private double P = DEFAULT_P; // default = 10
    private double D = DEFAULT_D; // default = 0
    private double I = DEFAULT_I; // default = 3
    private double F = DEFAULT_F; // default = 0

    // DriveTrain physical constants
    private final double MAX_MOTOR_SPEED_RPS = 312.0 / 60.0;
    public final double TICKS_PER_REV = 537.7;
    private final double DRIVE_GEAR_REDUCTION = 1.0;
    private final double WHEEL_DIAMETER_INCHES = 3.93701;
    private final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    private double leftFrontTargetSpeed;
    private double rightFrontTargetSpeed;
    private double leftBackTargetSpeed;
    private double rightBackTargetSpeed;

    public static Params PARAMS;

    public MecanumKinematics kinematics;
    public MotorFeedforward feedforward;
    public TurnConstraints defaultTurnConstraints;
    public VelConstraint defaultVelConstraint;
    public AccelConstraint defaultAccelConstraint;

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;

    public VoltageSensor voltageSensor;

    public Localizer localizer;
    public Pose2d pose;
    private IMU imu;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftRear, rightRear, rightFront;

        private int lastLeftFrontPos, lastLeftRearPos, lastRightRearPos, lastRightFrontPos;
        private Rotation2d lastHeading;

        public DriveLocalizer() {

            RawEncoder LFEncoder = new RawEncoder(MecanumDriveMona.this.leftFront);
            RawEncoder LBEncoder = new RawEncoder(MecanumDriveMona.this.leftBack);
            RawEncoder RFEncoder = new RawEncoder(MecanumDriveMona.this.rightFront);
            RawEncoder RBEncoder = new RawEncoder(MecanumDriveMona.this.rightBack);

            LFEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            LBEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            RFEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
            RBEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

            leftFront = new OverflowEncoder(LFEncoder);
            leftRear = new OverflowEncoder(LBEncoder);
            rightRear = new OverflowEncoder(RBEncoder);
            rightFront = new OverflowEncoder(RFEncoder);

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftRearPos = leftRear.getPositionAndVelocity().position;
            lastRightRearPos = rightRear.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;

            lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
            PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftRearPosVel.position - lastLeftRearPos),
                            leftRearPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightRearPosVel.position - lastRightRearPos),
                            rightRearPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftRearPos = leftRearPosVel.position;
            lastRightRearPos = rightRearPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDriveMona() {
    }

    public void init() {

        HardwareMap hardwareMap = Robot.getInstance().getActiveOpMode().hardwareMap;
        pose = new Pose2d(0,0,0);
        imu = Robot.getInstance().getGyro().getIMU();

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
        leftFront.setVelocityPIDFCoefficients(P, I, D, F);
        rightFront.setVelocityPIDFCoefficients(P, I, D, F);
        leftBack.setVelocityPIDFCoefficients(P, I, D, F);
        rightBack.setVelocityPIDFCoefficients(P, I, D, F);

        Params PARAMS = new Params();

        //sets the parameters based on the robot we have set
        //right now we just have two different sets
        PARAMS.init();

        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));

        defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    (int) Math.ceil(t.path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

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

    private void drawPoseHistory(Canvas c) {
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

    private static void drawRobot(Canvas c, Pose2d t) {
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

    public void mecanumDriveSpeedControl() {

        if (drive==0 && strafe ==0 && turn==0) {
            //stop and reset encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //We might have to set power to zero, but lets see if it works without that now that we skip setting velocity to zero.

            //Put the encoders back in run to encoder mode
            //This should be unnecessary
//            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {

            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            //safetydrivespeedfactor is set when we lookforapriltags based on the closest backdrop apriltag we see
            if (Robot.getInstance().getVision().blueBackdropAprilTagFound &&
                    Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.RED &&
                    drive > .1) {
                drive = Math.min(drive, Robot.getInstance().getDriveController().safetyDriveSpeedFactor);
            }
            //If we see red tags and we are blue and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            else if (Robot.getInstance().getVision().redBackdropAprilTagFound &&
                    Robot.getInstance().getVision().getInitVisionProcessor().allianceColorFinal == InitVisionProcessor.AllianceColor.BLUE &&
                    drive > .1) {
                drive = Math.min(drive, Robot.getInstance().getDriveController().safetyDriveSpeedFactor);
            }

            double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
            double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
            double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

            leftFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
            rightFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
            leftBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
            rightBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

            leftFront.setVelocity(leftFrontTargetSpeed);
            rightFront.setVelocity(rightFrontTargetSpeed);
            leftBack.setVelocity(leftBackTargetSpeed);
            rightBack.setVelocity(rightBackTargetSpeed);
        }
    }

    public void mecanumDrivePowerControl (){

        // Put Mecanum Drive math and motor commands here.
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


    public void telemetryDriveTrain() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");

        Robot.getInstance().getActiveOpMode().telemetry.addData("Drive: ", drive);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Strafe: ", strafe);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Turn: ", turn);

        double targetSpeedLF = Math.round(100.0 * leftFrontTargetSpeed / TICKS_PER_REV);
        double targetSpeedRF = Math.round(100.0 * rightFrontTargetSpeed / TICKS_PER_REV);
        double targetSpeedLB = Math.round(100.0 * leftBackTargetSpeed / TICKS_PER_REV);
        double targetSpeedRB = Math.round(100.0 * rightBackTargetSpeed / TICKS_PER_REV);

        double actualSpeedLF = Math.round(100.0 * leftFront.getVelocity() / TICKS_PER_REV);
        double actualSpeedRF = Math.round(100.0 * rightFront.getVelocity() / TICKS_PER_REV);
        double actualSpeedLB = Math.round(100.0 * leftBack.getVelocity() / TICKS_PER_REV);
        double actualSpeedRB = Math.round(100.0 * rightBack.getVelocity() / TICKS_PER_REV);

        Robot.getInstance().getActiveOpMode().telemetry.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * leftFront.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("RF" + " Speed: " + JavaUtil.formatNumber(actualSpeedRF, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRF, 4, 1) + " " + "Power: " + Math.round(100.0 * rightFront.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("LB" + " Speed: " + JavaUtil.formatNumber(actualSpeedLB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedLB, 4, 1) + " " + "Power: " + Math.round(100.0 * leftBack.getPower()) / 100.0);
        Robot.getInstance().getActiveOpMode().telemetry.addLine("RB" + " Speed: " + JavaUtil.formatNumber(actualSpeedRB, 4, 1) + "/" + JavaUtil.formatNumber(targetSpeedRB, 4, 1) + " " + "Power: " + Math.round(100.0 * rightBack.getPower()) / 100.0);
    }

    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    public void setMotorPower (double lF, double rF, double lB, double rB){
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        leftBack.setPower(lB);
        rightBack.setPower(rB);
    }

    public class DrawCurrentPosition implements Action {

        public boolean run(@NonNull TelemetryPacket p) {

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            return false;
        }
    }


}

