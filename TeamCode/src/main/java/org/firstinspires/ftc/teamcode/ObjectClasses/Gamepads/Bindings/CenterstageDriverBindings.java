package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateEndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;

public class CenterstageDriverBindings {

    public CenterstageDriverBindings(GamepadEx gamepad) {
            gamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.OPEN))
                    .whenReleased(new ActuateEndEffector(Robot.getInstance().getEndEffectorSubsystem(), EndEffectorSubsystem.EndEffectorStates.CLOSED));

            gamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new RotateShoulder(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.INTAKE))
                    .whenReleased(new RotateShoulder(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.BACKDROP));

            gamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(new MoveLiftSlide(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.MID))
                    .whenReleased(new MoveLiftSlide(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.HOME));


        }

    }
