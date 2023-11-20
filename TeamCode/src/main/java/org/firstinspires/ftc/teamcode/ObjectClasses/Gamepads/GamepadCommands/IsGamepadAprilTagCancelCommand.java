package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class IsGamepadAprilTagCancelCommand extends CommandBase {

    boolean done;
    GamepadEx gamepad;
    public IsGamepadAprilTagCancelCommand(GamepadEx pad)
    {
        gamepad=pad;
    }

    @Override
    public void initialize() {
        done=false;
    }

    @Override
    public void execute(){
        done = Robot.getInstance().getDriveSubsystem().driverGamepadCancellingAprilTagDriving(
                gamepad.getLeftY());
    }

    @Override
    public boolean isFinished(){
       if (done)
       {
           return true;
       } else return false;
    }
}
