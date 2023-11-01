package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakeState;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakeStateVelocity;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class IntakeTestingDriverBindings {
    public TriggerReader rightTrigger;

    public IntakeTestingDriverBindings(GamepadEx gamepad) {
        rightTrigger = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ChangeIntakeState(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON))
                .whenReleased(new ChangeIntakeState(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ChangeIntakeState(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
                .whenReleased(new ChangeIntakeState(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ChangeIntakeStateVelocity(intakeSubsystem,
                        IntakeSubsystem.IntakeStates.INTAKE_REVERSE,
                        GamepadHandling.getDriverGamepad()::getRightY))
                .whenReleased(new ChangeIntakeState(intakeSubsystem,
                        IntakeSubsystem.IntakeStates.INTAKE_OFF));

    }
}
