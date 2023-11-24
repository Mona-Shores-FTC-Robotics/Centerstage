package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.RED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem.DeliverLocation.RIGHT;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.ActuateGripperAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.MoveLiftSlideAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmActions.RotateShoulderAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDropAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

import java.util.Arrays;

@Disabled
@Autonomous(name = "TEST - Just AutoDriveToBackdrop")
public class Test_AutoDriveToBackdropAction extends LinearOpMode {

    private static final VisionSubsystem.DeliverLocation DELIVER_LOCATION= LEFT;
    public double FORWARD_SCORE_DISTANCE= 5.5;
    public Action forwardAction;
    public Action backwardAction;

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        // Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_CENTERSTAGE);

        // Initialize Gamepad and Robot
        Robot.getInstance().init(Robot.OpModeType.AUTO, gamepadHandling);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing();
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing();
        telemetry.update();

        //Set the starting pose of the robot
//        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        while(!Robot.getInstance().getVisionSubsystem().getVisionPortal().getProcessorEnabled(
                Robot.getInstance().getVisionSubsystem().getAprilTagProcessor()))
        {
            //wait for april tag processor to be enabled
        }

        telemetry.clearAll();

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //Start the Auto Timer
        MatchConfig.OpModeTimer = new ElapsedTime();
        MatchConfig.OpModeTimer.reset();

        //Start the Loop Timer
        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();

        //Start the April Tag timestamp timer
        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        SequentialAction autoDriveAndScore =
                new SequentialAction(
                        new AutoDriveToBackDropAction(DELIVER_LOCATION),
                        ScorePixelAction(DELIVER_LOCATION));

        Actions.runBlocking( autoDriveAndScore);
    }

    public Action ScorePixelAction(VisionSubsystem.DeliverLocation deliverLocation) {
        SequentialAction scorePixel =
                new SequentialAction(
                        new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                        new ParallelAction(
                            new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.SAFE),
                            new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.BACKDROP)
                                ),
                        new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.AUTO_LOW),
                        MoveRobotForward(deliverLocation),
                        new ActuateGripperAction(GripperSubsystem.GripperStates.OPEN),
                        new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.AUTO_HIGH),
                        MoveRobotBackwards(deliverLocation),
                        new ParallelAction(
                                new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.SAFE),
                                new ActuateGripperAction(GripperSubsystem.GripperStates.CLOSED),
                                new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.HALFWAY)
                        ),
                        new ParallelAction(
                                new RotateShoulderAction(ShoulderSubsystem.ShoulderStates.INTAKE),
                                new MoveLiftSlideAction(LiftSlideSubsystem.LiftStates.HOME)
                        )
                );
        return scorePixel;
    }

    private Action MoveRobotForward(VisionSubsystem.DeliverLocation deliverLocation) {
        Pose2d beginPose;
        if (MatchConfig.finalAllianceColor==RED)
        {
            if (deliverLocation==LEFT)
            {
                beginPose= FieldConstants.RED_BACKDROP_LEFT;
            } else if (deliverLocation==RIGHT)
            {
                beginPose= FieldConstants.RED_BACKDROP_RIGHT;
            } else {
                beginPose = FieldConstants.RED_BACKDROP_CENTER;
            }

        } else
        {
            if (deliverLocation==LEFT)
            {
                beginPose= FieldConstants.BLUE_BACKDROP_LEFT;
            } else if (deliverLocation==RIGHT)
            {
                beginPose= FieldConstants.BLUE_BACKDROP_RIGHT;
            } else {
                beginPose = FieldConstants.BLUE_BACKDROP_CENTER;
            }
        }

        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        VelConstraint overrideVelConstraint;
        AccelConstraint overrideAccelConstraint;
        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(10)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-10, 10);

        Pose2d endPose = new Pose2d(beginPose.position.x+FORWARD_SCORE_DISTANCE,
                             beginPose.position.y,
                                beginPose.heading.log());

        return drive.actionBuilder(beginPose)
                .lineToX(endPose.position.x, overrideVelConstraint, overrideAccelConstraint)
                .build();
    }

    private Action MoveRobotBackwards(VisionSubsystem.DeliverLocation deliverLocation) {
        Pose2d beginPose;
        if (MatchConfig.finalAllianceColor==RED)
        {
            if (deliverLocation==LEFT)
            {
                beginPose= FieldConstants.RED_BACKDROP_LEFT;
            } else if (deliverLocation==RIGHT)
            {
                beginPose= FieldConstants.RED_BACKDROP_RIGHT;
            } else {
                beginPose = FieldConstants.RED_BACKDROP_CENTER;
            }

        } else
        {
            if (deliverLocation==LEFT)
            {
                beginPose= FieldConstants.BLUE_BACKDROP_LEFT;
            } else if (deliverLocation==RIGHT)
            {
                beginPose= FieldConstants.BLUE_BACKDROP_RIGHT;
            } else {
                beginPose = FieldConstants.BLUE_BACKDROP_CENTER;
            }
        }

        MecanumDriveMona drive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        VelConstraint overrideVelConstraint;
        AccelConstraint overrideAccelConstraint;
        overrideVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(10)
                ));

        overrideAccelConstraint = new ProfileAccelConstraint(-10, 10);

        Pose2d startingBackupPose = new Pose2d(beginPose.position.x+FORWARD_SCORE_DISTANCE, beginPose.position.y, beginPose.heading.log());

        return drive.actionBuilder(startingBackupPose)
                .setReversed(true)
                .lineToX(beginPose.position.x-3, overrideVelConstraint, overrideAccelConstraint)
                .build();
    }
}

