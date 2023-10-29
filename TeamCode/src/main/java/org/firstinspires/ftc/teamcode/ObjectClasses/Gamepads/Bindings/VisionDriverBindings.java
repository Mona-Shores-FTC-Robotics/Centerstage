package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo0;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo180;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo270;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands.turnTo90;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class VisionDriverBindings {

    public VisionDriverBindings(GamepadEx gamepad) {

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(turnTo0);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(turnTo180);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(turnTo90);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(turnTo270);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(turnTo0);

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(backUpAndRotate, true);


    }

    }
