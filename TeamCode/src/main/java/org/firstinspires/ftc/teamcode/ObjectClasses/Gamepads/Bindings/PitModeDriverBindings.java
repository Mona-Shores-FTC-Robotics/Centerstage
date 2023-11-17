package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Constants.FieldConstants.END_GAME_TIME;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateGripperCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ChangeWinchPowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReadyClimberArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReleaseDroneCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class PitModeDriverBindings {



    ClimberSubsystem climberSubsystem = Robot.getInstance().getClimberSubsystem();
    ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
    GripperSubsystem gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();

    public PitModeDriverBindings(GamepadEx gamepad) {

        //////////////////////////////////////////////////////////
        //                                                      //
        // START TO FLY DRONE                                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.FLY));

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT-BUMPER TO ARM DRONE                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.HOLD));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP - WIND WINCH                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_UP))
                .whenReleased(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.OFF));


        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN - UNWIND WINCH                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_DOWN))
                .whenReleased(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.OFF));

        //////////////////////////////////////////////////////////
        //                                                      //
        // A BUTTON - SET ALL SERVOS TO NEUTRAL (0.5)           //
        //    ALL SERVOS EXCEPT DRONE SERVO                     //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                                new ReadyClimberArmCommand(climberSubsystem, ClimberSubsystem.ClimberArmStates.STOWED),
                                new RotateShoulderCommand(shoulderSubsystem, ShoulderSubsystem.ShoulderStates.REST),
                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.REST))
                        );


    }
}
