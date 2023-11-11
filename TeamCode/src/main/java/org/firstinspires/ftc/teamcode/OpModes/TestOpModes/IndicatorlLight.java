package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Idicator Light Test")

public class IndicatorlLight extends LinearOpMode {

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


        while (opModeIsActive()) {
            if (touch.getState()) {

                green.setState(false);
                red.setState(true);
            } else {
                red.setState(false);
                green.setState(true);
            }
        }
    }
}
