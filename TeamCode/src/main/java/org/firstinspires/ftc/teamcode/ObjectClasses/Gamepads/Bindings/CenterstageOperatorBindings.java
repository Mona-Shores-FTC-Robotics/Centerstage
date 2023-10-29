package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.CenterstageCommands;
import org.firstinspires.ftc.teamcode.ObjectClasses.Commands.IntakeCommands.ChangeIntakeState;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.VisionSubsystem;

public class CenterstageOperatorBindings {

    public CenterstageOperatorBindings(GamepadEx operatorGamepad) {

        VisionSubsystem visionSubsystem = Robot.getInstance().getVisionSubsystem();

        /** X Button **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand( ()-> {
                    visionSubsystem.setDeliverLocation(VisionSubsystem.DeliverLocation.LEFT);}));

        /** Y Button **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand( ()-> {
                    visionSubsystem.setDeliverLocation(VisionSubsystem.DeliverLocation.CENTER);}));

        /** B Button **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand( ()-> {
                    visionSubsystem.setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);}));

        /** A Button **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(CenterstageCommands::MakeScorePixelCommand);


        //can we make one of the commands in our sequential command group a variable and then use the bumpers
        //to change which command the extension uses?
        /** Right Bumper **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand( ()-> {
                    visionSubsystem.setDeliverHeight(VisionSubsystem.DeliverHeight.MID);}));

        /** Left Bumper **/
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand( ()-> {
                    visionSubsystem.setDeliverHeight(VisionSubsystem.DeliverHeight.HIGH);}));

    }
}
