package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name = "NEW Idicator Light Test")

public class IndicatorlLightNew extends LinearOpMode {

    public Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = Robot.createInstance(this, Robot.RobotType.ROBOT_INDICATOR_LIGHT, Robot.OpModeType.TELEOP);

        robot.init();

        waitForStart();


        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
