package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name = "Basic_Auto")
public class Basic_Auto extends LinearOpMode {

    Robot robot = Robot.createInstance(this);
    MecanumDrive roadRunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    private InitVisionProcessor.TeamPropLocation teamPropLocationAfterInit = InitVisionProcessor.TeamPropLocation.CENTER;
    private InitVisionProcessor.AllianceColor allianceColorAfterInit = InitVisionProcessor.AllianceColor.BLUE;

    private final ElapsedTime runtime = new ElapsedTime();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            teamPropLocationAfterInit = robot.getVision().getInitVisionProcessor().getTeamPropLocationFinal();
            allianceColorAfterInit = robot.getVision().getInitVisionProcessor().getAllianceColorFinal();

            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            telemetry.addData("Alliance Color", robot.getVision().getInitVisionProcessor().getTeamPropLocationFinal());
            telemetry.addData("Team Prop Location", robot.getVision().getInitVisionProcessor().getTeamPropLocationFinal());
            telemetry.addData("left Square Blue/Red Percent", robot.getVision().getInitVisionProcessor().getLeftPercent());
            telemetry.addData("Middle Square Blue/Red Percent", robot.getVision().getInitVisionProcessor().getCemterPercent());
            telemetry.addData("Right Square Blue/Red Percent", robot.getVision().getInitVisionProcessor().getRightPercent());
            telemetry.update();
        }
        telemetry.addData("Team Prop Location After Init", teamPropLocationAfterInit);
        telemetry.addData("Alliance Color After Init", allianceColorAfterInit);
        telemetry.update();

        runtime.reset();

        Actions.runBlocking(
                    roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        }
    }

