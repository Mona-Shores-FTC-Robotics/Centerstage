package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.ActuateEndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;

public class OperatorAutoInitBindings {

    public OperatorAutoInitBindings(GamepadEx operatorGamepad, Robot.RobotType robotType) {
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.OPEN))
//                .whenReleased(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.CLOSED));
    }
}
