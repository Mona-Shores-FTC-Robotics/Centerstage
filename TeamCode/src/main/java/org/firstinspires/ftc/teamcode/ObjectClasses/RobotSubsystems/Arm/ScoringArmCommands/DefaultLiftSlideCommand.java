package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

import java.util.function.DoubleSupplier;

//This is a default command to move the lift with the gamepad stick
public class DefaultLiftSlideCommand extends CommandBase {
    //Declare the local variable to hold the liftsubsystem

    //Declare the local variable liftSupplier to hold the lift stick - can you figure out what type it should be?

    public DefaultLiftSlideCommand(LiftSlideSubsystem subsystem, DoubleSupplier liftStick) {
        //save the input subsystem to the local variable

        //save the liftStick as the liftSupplier

        //add the subsystem to the requirements
    }

    @Override
    public void initialize() {
        //Set the target ticks of the subsystem to the current ticks of the subsystem so that movements are relative to where the lift is right now

    }

    public void execute() {
        //if liftSupplier is positive and above deadzone...

        {
            //then set the currentState of the subsystem to MANUAL... and

            //then set the power of the lift to EXTENSION_LIFT_POWER

        }

        //else if liftSupplier is negative and lower than deadzone...
        {
            //then set the currentState of the subsystem to MANUAL... and

            //then set the power of the lift to RETRACTION_LIFT_POWER
        }

        //else if we are in the dead zone we should set extension lift power to maintain our position
         {

        }

        //Use the liftSupplier to calculate the new target based on the liftSupplier - just uncomment this line, another student will write setTargetTicks() method
        //liftSlideSubsystem.setTargetTicks(calculateNewTargetTicks(liftSupplier));

        //set the target position of the subsystem to the new ticks value

        //set the lift motor to RUN TO POSITION - this might not be necessary

    }

    @Override
    public boolean isFinished() {
        //this command should never finish
        return false;
    }

    //Write a private method that returns an integer based on the left stick
    //The name of the method should be calculateNewTargetTicks and it should receive as input a DoubleSupplier variable
    //try using chatGPT to write this method
    private int calculateNewTargetTicks(DoubleSupplier leftStick)
    {
        //get the double from the double supplier by (e.g., liftSupplier.getAsDouble())

        //multiply the double by a scaling factor (SCALE_FACTOR_FOR_MANUAL_LIFT) - scaling factor is availbale in the subsystem parameters.
        double scaledStick =  leftStick.getAsDouble()*SCALE_FACTOR_FOR_MANUAL_LIFT;
        //round the result and cast as an integer so that we have the change in ticks (this could be a positive or negative number)
        int deltaTicks = (int) Math.round(scaledStick);
        //add the result to the current ticks to get the new tickTarget
//        int newTicks = liftSubsystem.getcurrentticks()+deltaTicks;
        // Ensure the new target is within the range of MIN_TICKS to MAX_TICKS using Range.clip

        //return the result
return 1;
    }
}
