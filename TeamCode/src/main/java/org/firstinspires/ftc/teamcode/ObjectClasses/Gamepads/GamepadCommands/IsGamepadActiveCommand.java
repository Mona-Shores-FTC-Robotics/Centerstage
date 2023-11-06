package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;

public class IsGamepadActiveCommand extends CommandBase {

    boolean done;

    @Override
    public void initialize() {
        done=false;
    }

    @Override
    public void execute(){
        done = GamepadHandling.driverGamepadIsActive(   GamepadHandling.getDriverGamepad().getLeftY(),
                                                        GamepadHandling.getDriverGamepad().getLeftX(),
                                                        GamepadHandling.getDriverGamepad().getRightX());
    }

    @Override
    public boolean isFinished(){
       if (done)
       {
           return true;
       } else return false;
    }
}
