package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.AutoDriveToBackDrop;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionTelemetry;

@Autonomous(name = "Alignment Test Auto")
public class Alightment_Test_Auto extends LinearOpMode {

    private MecanumDriveMona roadRunnerDriveSubsystem;

    private InitVisionProcessor.TeamPropLocation teamPropLoc;
    private InitVisionProcessor.AllianceColor allianceColor;
    private InitVisionProcessor.SideOfField sideOfField;

    private Action selectedRoute;

    @Override
    public void runOpMode() {
        /** Create and Initialize the robot **/
        Robot.createInstance(this, Robot.RobotType.ROBOT_VISION);

        /** Initialize Gamepad and Robot - Order Important **/
        GamepadHandling.init();
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getVisionSubsystem().SwitchToInitVisionProcessor();

        roadRunnerDriveSubsystem = Robot.getInstance().getDriveSubsystem().mecanumDrive;

        Action testAutoAlignAction = new AutoDriveToBackDrop();

        while (opModeInInit()) {
            // Add Vision Init Processor Telemetry
            VisionTelemetry.telemetryForInitProcessing();
            GamepadHandling.getDriverGamepad().readButtons();
            GamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Reset Gyro
        Robot.getInstance().getGyroSubsystem().resetAbsoluteYaw();

        //Display the initVision telemetry a final time
        VisionTelemetry.telemetryForInitProcessing();
        telemetry.update();

        teamPropLoc = MatchConfig.finalTeamPropLocation;
        allianceColor = MatchConfig.finalAllianceColor;
        sideOfField = MatchConfig.finalSideOfField;

        //Set the starting pose of the robot
        Robot.getInstance().getVisionSubsystem().setStartingPose(allianceColor, sideOfField);

        //this saves the alliance color in a spot that persists between opModes
        MatchConfig.finalAllianceColor = allianceColor;

        //After Init switch the vision processing to AprilTags
        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        while(!Robot.getInstance().getVisionSubsystem().getVisionPortal().getProcessorEnabled(
                Robot.getInstance().getVisionSubsystem().getAprilTagProcessor()))
        {
            //wait for april tag processor to be enabled
        };

        telemetry.clearAll();

        Actions.runBlocking(testAutoAlignAction);
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getDriveSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getGyroSubsystem());
        CommandScheduler.getInstance().unregisterSubsystem(Robot.getInstance().getVisionSubsystem());
        Robot.reset();
    }

}

