package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.IntakeCommands.ChangeIntakeState;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.ActuateEndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.MoveLiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.ScoringArmCommands.RotateShoulder;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ArmSubsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IntakeSubsystem;

public class DriverTeleOpBindings {

    public DriverTeleOpBindings(GamepadEx driverGamepad, Robot.RobotType robotType) {

        switch(robotType){

            case ROBOT_INTAKE:
            {
                //for testing
                intakeBindings(driverGamepad);
                break;
            }
            case ROBOT_SCORING_ARM:
            {
                //for testing
                scoringArmBindings(driverGamepad);
                break;
            }
            case ROBOT_VISION:
            case ROBOT_DRIVE_BASE:
            case ROBOT_CENTERSTAGE:
            {
                drivingBindings(driverGamepad);
                break;
            }
        }
    }

    private void intakeBindings(GamepadEx gamepad) {
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ChangeIntakeState(Robot.getInstance().getIntakeSubsystem(), IntakeSubsystem.IntakeStates.INTAKE_ON))
                .whenReleased(new ChangeIntakeState(Robot.getInstance().getIntakeSubsystem(), IntakeSubsystem.IntakeStates.INTAKE_OFF));
    }

    private void drivingBindings(GamepadEx driverGamepad) {
    }

    private void scoringArmBindings(GamepadEx gamepad) {
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
