package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR19429;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR20245;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;

public class DriveLocalizer implements Localizer {
    public final Encoder leftFront, leftRear, rightRear, rightFront;

    private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;

    public DriveLocalizer(MecanumDriveMona mecanumDrive) {

        RawEncoder LFEncoder = new RawEncoder(mecanumDrive.leftFront);
        RawEncoder LBEncoder = new RawEncoder(mecanumDrive.leftBack);
        RawEncoder RFEncoder = new RawEncoder(mecanumDrive.rightFront);
        RawEncoder RBEncoder = new RawEncoder(mecanumDrive.rightBack);

        LFEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        LBEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        RFEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        RBEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront = new OverflowEncoder(LFEncoder);
        leftRear = new OverflowEncoder(LBEncoder);
        rightRear = new OverflowEncoder(RBEncoder);
        rightFront = new OverflowEncoder(RFEncoder);

        lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
        lastLeftBackPos = leftRear.getPositionAndVelocity().position;
        lastRightBackPos = rightRear.getPositionAndVelocity().position;
        lastRightFrontPos = rightFront.getPositionAndVelocity().position;

        double headingRadians = Robot.getInstance().getGyroSubsystem().imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        lastHeading = Rotation2d.exp(headingRadians);
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
        PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        double headingRadians = Robot.getInstance().getGyroSubsystem().getCurrentRelativeYawRadians();
        double headingDegrees = Math.toDegrees(headingRadians);
        Rotation2d heading = Rotation2d.exp(Robot.getInstance().getGyroSubsystem().imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist;
        if (MatchConfig.robot19429) {

             twist = Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(MotorParametersRR19429.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftRearPosVel.position - lastLeftBackPos),
                            leftRearPosVel.velocity,
                    }).times(MotorParametersRR19429.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightRearPosVel.position - lastRightBackPos),
                            rightRearPosVel.velocity,
                    }).times(MotorParametersRR19429.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(MotorParametersRR19429.inPerTick)
            ));
        } else
        {
            twist = Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(MotorParametersRR20245.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftRearPosVel.position - lastLeftBackPos),
                            leftRearPosVel.velocity,
                    }).times(MotorParametersRR20245.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightRearPosVel.position - lastRightBackPos),
                            rightRearPosVel.velocity,
                    }).times(MotorParametersRR20245.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(MotorParametersRR20245.inPerTick)
            ));
        }

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftRearPosVel.position;
        lastRightBackPos = rightRearPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        return new Twist2dDual<>(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }
}