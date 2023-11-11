package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakePowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakeVelocityCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class IntakeTestingDriverBindings {
    public TriggerReader rightTrigger;

    public IntakeTestingDriverBindings(GamepadEx gamepad) {
        rightTrigger = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ChangeIntakeVelocityCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON))
                .whenReleased(new ChangeIntakeVelocityCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ChangeIntakeVelocityCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
                .whenReleased(new ChangeIntakeVelocityCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON))
                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

    }
}
