package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.LiftSlideParameters.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem.liftSlideParameters;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultLiftSlideCommand extends CommandBase {
    private int previousCurrentTicks;

    //Declare the local variable to hold the liftsubsystem
    private final LiftSlideSubsystem liftSlideSubsystem;

    //Declare the local variable to hold the lift stick
    private DoubleSupplier liftSupplier;

    public DefaultLiftSlideCommand(LiftSlideSubsystem subsystem, DoubleSupplier liftStick) {
        liftSlideSubsystem = subsystem;
        liftSupplier = liftStick;

        //add the subsystem to the requirements
        addRequirements(liftSlideSubsystem);
    }

    @Override
    public void initialize() {
        //Set the target ticks to the current ticks so that movements are relative to where the lift is right now
        previousCurrentTicks = liftSlideSubsystem.getCurrentTicks();
        liftSlideSubsystem.setTargetTicks(previousCurrentTicks);
        liftSlideSubsystem.setCurrentState(LiftSlideSubsystem.LiftStates.MANUAL);
        liftSlideSubsystem.setTargetState(LiftSlideSubsystem.LiftStates.MANUAL);
    }

    public void execute() {
        //Use the liftSupplier to calculate a new target based on the liftSupplier
        liftSlideSubsystem.setTargetTicks(calculateNewTargetTicks(liftSupplier));

        boolean inSafeZone = liftSlideSubsystem.getTargetTicks() > LiftSlideSubsystem.liftSlideParameters.SAFE_ZONE_FOR_MANUAL_LIFT;

        if (inSafeZone)
        {
            //if liftSupplier is positive and above deadzone, then use Extension power
            if (liftSlideSubsystem.getCurrentTicks() <= liftSlideSubsystem.getTargetTicks()-LiftSlideSubsystem.liftSlideParameters.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
                liftSlideSubsystem.liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P, liftSlideParameters.VEL_I, liftSlideParameters.VEL_D, liftSlideParameters.VEL_F);
                liftSlideSubsystem.liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);
                liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
            }

            //if liftSupplier is negative and lower than deadzone, then use Retraction power
            else if (liftSlideSubsystem.getCurrentTicks() > liftSlideSubsystem.getTargetTicks() + LiftSlideSubsystem.liftSlideParameters.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
                liftSlideSubsystem.liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P_DOWN, liftSlideParameters.VEL_I_DOWN, liftSlideParameters.VEL_D_DOWN, liftSlideParameters.VEL_F_DOWN);
                liftSlideSubsystem.liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P_DOWN);
                liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.RETRACTION_LIFT_POWER);
            } else
            {
                liftSlideSubsystem.liftSlide.setVelocityPIDFCoefficients(liftSlideParameters.VEL_P, liftSlideParameters.VEL_I, liftSlideParameters.VEL_D, liftSlideParameters.VEL_F);
                liftSlideSubsystem.liftSlide.setPositionPIDFCoefficients(liftSlideParameters.POS_P);
                liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
            }

        //set the target position to the new ticks value
        liftSlideSubsystem.liftSlide.setTargetPosition(liftSlideSubsystem.getTargetTicks());
        }
//        else if (!inSafeZone &&  liftSlideSubsystem.getTargetTicks() > LiftSlideSubsystem.liftSlideParameters.SAFE_ZONE_FOR_MANUAL_LIFT-200)
//        {
//            //if liftSupplier is positive and above deadzone, then use Extension power
//            if (liftSlideSubsystem.getCurrentTicks() <= liftSlideSubsystem.getTargetTicks()-LiftSlideSubsystem.liftSlideParameters.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
//                liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
//            }
//        }
        else
        {
            liftSlideSubsystem.setTargetTicks(previousCurrentTicks);
            liftSlideSubsystem.liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
        }
        previousCurrentTicks = liftSlideSubsystem.getTargetTicks();
    }

    @Override
    public boolean isFinished() {
        //this command should never finish
        return false;
    }

    private int calculateNewTargetTicks(DoubleSupplier liftSupplier) {
        // Update the targetTicks based on the liftSupplier's value
        int deltaTicks = (int) Math.round(liftSupplier.getAsDouble()*LiftSlideSubsystem.liftSlideParameters.SCALE_FACTOR_FOR_MANUAL_LIFT);

        int newTargetTicks = previousCurrentTicks + deltaTicks;

        // Ensure the new target is within the range of MIN_TICKS to MAX_TICKS using Range.clip
        int clippedNewTargetTicks = (int) Range.clip(newTargetTicks, liftSlideSubsystem.MIN_TARGET_TICKS, liftSlideSubsystem.MAX_TARGET_TICKS);

        return clippedNewTargetTicks;
    }
}
