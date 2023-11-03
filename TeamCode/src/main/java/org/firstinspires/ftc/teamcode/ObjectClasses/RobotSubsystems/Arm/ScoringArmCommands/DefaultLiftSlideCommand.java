package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

import java.util.function.DoubleSupplier;

//This is a default command to move the lift with the gamepad stick
public class DefaultLiftSlideCommand extends CommandBase {
    //Declare the local variable to hold the liftsubsystem
    public LiftSlideSubsystem liftSlideSubsystem;
    //Declare the local variable liftSupplier to hold the lift stick - can you figure out what type it should be?
    public DoubleSupplier liftSupplier;
    public DefaultLiftSlideCommand(LiftSlideSubsystem subsystem, DoubleSupplier liftStick) {
        //save the input subsystem to the local variable
        liftSlideSubsystem = subsystem;
        //save the liftStick as the liftSupplier
        liftSupplier = liftStick;
        //add the subsystem to the requirements
        addRequirements(liftSlideSubsystem);
    }

    @Override
    public void initialize() {
        //Set the target ticks of the subsystem to the current ticks of the subsystem so that movements are relative to where the lift is right now
        liftSlideSubsystem.setTargetTicks(liftSlideSubsystem.getCurrentTicks());
    }

    public void execute() {
        //if liftSupplier is positive and above deadzone...
        if (liftSupplier.getAsDouble() > .1)
        {
            //then set the currentState of the subsystem to MANUAL... and
            liftSlideSubsystem.setCurrentState(LiftSlideSubsystem.LiftStates.MANUAL);
            //then set the power of the lift to EXTENSION_LIFT_POWER
            liftSlideSubsystem.liftSlide.setPower(EXTENSION_LIFT_POWER);
        }

        //else if liftSupplier is negative and lower than deadzone...
        else if (liftSupplier.getAsDouble() < .1)
        {
            //then set the currentState of the subsystem to MANUAL... and
            liftSlideSubsystem.setCurrentState(LiftSlideSubsystem.LiftStates.MANUAL);
            //then set the power of the lift to RETRACTION_LIFT_POWER
            liftSlideSubsystem.liftSlide.setPower(RETRACTION_LIFT_POWER);
        }

        //else if we are in the dead zone we should set extension lift power to maintain our position
        else if (liftSupplier.getAsDouble() == .1)
         {
             liftSlideSubsystem.liftSlide.getCurrentPosition();

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

    {
        //get the double from the double supplier by (e.g., liftSupplier.getAsDouble())

        //multiply the double by a scaling factor (SCALE_FACTOR_FOR_MANUAL_LIFT) - scaling factor is availbale in the subsystem parameters.

        //round the result and cast as an integer so that we have the change in ticks (this could be a positive or negative number)

        //add the result to the current ticks to get the new tickTarget

        // Ensure the new target is within the range of MIN_TICKS to MAX_TICKS using Range.clip

        //return the result

    }
}
