package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

public class ScoringArmTestingDriverBindings {
    public static TriggerReader rightTrigger;
    public ScoringArmTestingDriverBindings(GamepadEx gamepad) {
        rightTrigger = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

//        gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.OPEN))
//                .whenReleased(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.CLOSED));
//
//        gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new RotateShoulder(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.INTAKE))
//                .whenReleased(new RotateShoulder(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.BACKDROP));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.LOW));


        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.MID));


        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.HIGH));

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.HOME));

    }
    }
