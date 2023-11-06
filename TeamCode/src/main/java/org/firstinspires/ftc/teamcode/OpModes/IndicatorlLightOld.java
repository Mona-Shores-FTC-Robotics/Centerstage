package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Idicator Light Test")

public class IndicatorlLightOld extends LinearOpMode {

    private DigitalChannel touch;
    private  DigitalChannel red;
    private DigitalChannel green;

    @Override
    public void runOpMode() throws InterruptedException {
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");

        touch = hardwareMap.get(DigitalChannel.class, "touch");

        waitForStart();

        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        red.setState(true);
        green.setState(true);



        while (opModeIsActive()) {
            if (touch.getState()) {

                green.setState(true);
                red.setState(false);
            } else {

                red.setState(false);
                green.setState(false);
            }
        }
    }
}
