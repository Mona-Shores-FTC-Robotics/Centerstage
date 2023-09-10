package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    DcMotorEx lift;
    LinearOpMode activeOpMode;
    private double MOTOR_SPEED_RPM = 435;
    private static double TICKS_PER_REV = 	((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * 28.0); //384.5
    private double MOTOR_SPEED_TPS = MOTOR_SPEED_RPM * TICKS_PER_REV / 60.0;
    private static double INCHES_PER_REV = 18.0 * 3.0 * 3.0 / 25.4; // pulley teeth * tooth spacing * lift stages / (mm/inch)
    public static double TICKS_PER_INCH = TICKS_PER_REV/INCHES_PER_REV;
    private double MAX_HEIGHT_INCHES = 30.0;
    private double outtakeShiftInches = 3.0;
    public int outtakeShiftTicks = (int) (outtakeShiftInches * TICKS_PER_INCH);

    public enum LiftHeights {
        JUNCTION_HIGH (35.6),
        JUNCTION_MEDIUM (25.7),
        JUNCTION_LOW (16.2),
        CONES_ONE (0.0),
        CONES_TWO (2.0),
        CONES_THREE (3.0),
        CONES_FOUR (4.0),
        CONES_FIVE (5.6),
        MANUAL (0),
        HOLD(0);
        private double inches;
        private int ticks;
        private LiftHeights(double inches) {
            this.inches = inches;
            this.ticks = (int) (inches * TICKS_PER_INCH);
        }
    }
    LiftHeights targetLiftHeight = LiftHeights.CONES_ONE;
    LiftHeights currentliftHeight = LiftHeights.CONES_ONE;

    private double MIN_SAFE_HEIGHT_INCHES = 9.0;
    public int MIN_SAFE_HEIGHT_TICKS = (int) Math.round(MIN_SAFE_HEIGHT_INCHES * TICKS_PER_INCH);
    public double SAFE_STOP_INCHES = 3.0;
    public int SAFE_STOP_TICKS = (int) (SAFE_STOP_INCHES*TICKS_PER_INCH);
    public double MAX_ALLOWED_DELTA_INCHES = .1;
    public int MAX_ALLOWED_DELTA_TICKS = 50;
    private double MIN_HOLD_HEIGHT_INCHES = LiftHeights.CONES_TWO.inches - 2* MAX_ALLOWED_DELTA_INCHES;
    private int MIN_HOLD_HEIGHT_TICKS = (int) (MIN_HOLD_HEIGHT_INCHES*TICKS_PER_INCH);
    private double HOLD_POWER = 0.9;
    private double RAISE_POWER = 0.9;
    private double LOWER_POWER = 0.0;
    private double STICK_DEAD_ZONE = 0.1;
    int deltaSafeLower = 0;

    int deltaLift = 0;
    double liftPower = 0;
    double lastLiftPower = 0;
    int runToPosition = 0;
    int lastRunToPosition = 0;
    int targetPosition = 0;
    public int currentPosition = 0;
    boolean safeToLower = false;
    boolean outputShifted = false;

    /* Constructor     */
    public Lift(LinearOpMode opMode) {
        activeOpMode = opMode;
    }

    /* Initialization */
    public void init (HardwareMap hwMap, LinearOpMode opMode){
        activeOpMode = opMode;
        lift = hwMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setPower(0);
       // lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    /**
     * Lift (targetPosition, currentPosition, minSafeHeight)
     * deltaLift = targetPosition - currentPosition
     * lastLiftPower = liftPower
     * lastRunToPosition = runToPosition
     * Case: abs(deltaLift) < MAX_ALLOWED_DELTA && targePosition > MIN_HOLD_HEIGHT
     * Set runToPosition = currentPosition
     * Set liftPower = HOLD_POWER
     * Case: abs(deltaLift) < MAX_ALLOWED_DELTA
     * Set runmode to stop and reset encoders
     * return
     * Case: deltaLift > 0
     * Set runToPosition = targetPosition
     * Set liftPower to = RAISE_POWER
     * Case: targetPosition > minSafeHeight
     * Set runToPosition = targetPosition
     * Set liftPower = DOWN_POWER
     * Else Case: // targetPosition < minSafeHeight
     * Set runToPosition = minSafeHeight
     * Set liftPower = DOWN_POWER
     * End Case
     * Case:  liftPower <> lastLiftPower
     * Set lift motor power to liftPower
     * End Case
     * Case: runToPosition <> lastRunToPosition
     * Set lift motor target position to runToPosition
     * Set lift motor to RUN_TO_POSITION
     * End Case
     */

    public void runLift (LiftHeights targetHeight, double manualInput, boolean liftHeightShift){
        targetPosition = targetHeight.ticks;
        if (targetHeight != LiftHeights.MANUAL && targetHeight != LiftHeights.HOLD) {
            deltaLift = targetHeight.ticks - currentPosition;
        } else {
            deltaLift = 0;
        }
        deltaSafeLower = MIN_SAFE_HEIGHT_TICKS + SAFE_STOP_TICKS - currentPosition;
        lastLiftPower = liftPower;
        lastRunToPosition = runToPosition;

        if (targetHeight == LiftHeights.MANUAL && Math.abs(manualInput) > STICK_DEAD_ZONE &&
                (safeToLower || currentPosition > MIN_SAFE_HEIGHT_TICKS)) {
            lift.setVelocity(manualInput * MOTOR_SPEED_TPS);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //lift.setPower(1.0);
            currentliftHeight = LiftHeights.MANUAL;
            outputShifted = false;
            return;
        } else if (targetHeight == LiftHeights.MANUAL && manualInput > STICK_DEAD_ZONE) {
            lift.setVelocity(manualInput * MOTOR_SPEED_TPS);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //lift.setPower(1.0);
            currentliftHeight = LiftHeights.MANUAL;
            outputShifted = false;
            return;
        } else if (liftHeightShift && !outputShifted) {
            runToPosition = runToPosition - outtakeShiftTicks;
            outputShifted = true;
        } else if (!liftHeightShift && outputShifted) {
            runToPosition = runToPosition + outtakeShiftTicks;
            outputShifted = false;
        } else if (currentliftHeight == LiftHeights.MANUAL) {
            runToPosition = currentPosition;
            liftPower = HOLD_POWER;
            currentliftHeight = LiftHeights.HOLD;
        } else if (Math.abs(deltaLift) < MAX_ALLOWED_DELTA_TICKS
                && (targetHeight == LiftHeights.CONES_ONE || targetHeight == LiftHeights.MANUAL)
                && currentPosition < MIN_HOLD_HEIGHT_TICKS) {
            liftPower = 0;
            lift.setPower(liftPower);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            currentliftHeight = targetHeight;
            return;
        } else if (deltaLift > MAX_ALLOWED_DELTA_TICKS || (!safeToLower && deltaSafeLower > 0)) {
            liftPower = RAISE_POWER;
            runToPosition = targetPosition;
        } else if (deltaLift < -MAX_ALLOWED_DELTA_TICKS && safeToLower) {
            liftPower = LOWER_POWER;
            runToPosition = targetPosition;
        } else if (deltaLift < -MAX_ALLOWED_DELTA_TICKS && deltaSafeLower < -MAX_ALLOWED_DELTA_TICKS) {
            liftPower = LOWER_POWER;
            runToPosition = MIN_SAFE_HEIGHT_TICKS + SAFE_STOP_TICKS;
        } else if (currentliftHeight != LiftHeights.HOLD){
            liftPower = HOLD_POWER;
            currentliftHeight = targetHeight;
            currentliftHeight = LiftHeights.HOLD;
        }

        if (currentliftHeight != LiftHeights.HOLD){
            currentliftHeight = targetHeight;
        }

        if (runToPosition > 0){
            lift.setTargetPosition(runToPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (currentliftHeight != LiftHeights.HOLD){
            liftPower = Math.min( liftPower * deltaLift / (3.0 * TICKS_PER_INCH),liftPower);
        }
        lift.setPower(liftPower);
        /* outtake timer never completes
        bouncy, especially when high **
        doesn't hold after manual lift **
        if stuck in auto score, can't move out of outtake **
        manual down = up **
        manual up = up **
        manual lift required to get above 0
         */
    }
}