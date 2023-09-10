package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Robot {
    private double STICK_DEADZONE = 0.1;
    LinearOpMode activeOpMode = null;
    DriveTrain mecDrive = new DriveTrain(activeOpMode);
    Lift lift = new Lift(activeOpMode);
    // ToDo Pass all values in constructors, not init, unless it might change based on opmode selected.
    Arm arm = new Arm(activeOpMode);
    Gyro gyro = new Gyro(activeOpMode);
    IntakeOuttake intake = new IntakeOuttake(activeOpMode);
    ElapsedTime runtime = new ElapsedTime();


    public enum AutoScoreSteps {
        INITIATE,
        WAIT_FOR_PRESS,
        OUTTAKE,
        RAISE_LIFT,
        ROTATE_AND_LOWER,
        COMPLETE;
    }
    AutoScoreSteps autoScoreState = AutoScoreSteps.COMPLETE;
    boolean autoScoreButtonPressed;
    Lift.LiftHeights targetLiftHeight = Lift.LiftHeights.CONES_ONE;
    boolean liftHeightShift = false;
    Arm.ArmPositions targetArmPosition = Arm.ArmPositions.INTAKE;
    IntakeOuttake.IntakeStates targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
    double autoScoreWait = 0;
    double autoScoreRaise = 0;
    double autoScoreFinal = 0;
    double autoScoreOuttake = 0;

    /* Constructor */
    public Robot(LinearOpMode opMode, ElapsedTime opModeRuntime) {
        activeOpMode = opMode;
        runtime = opModeRuntime;
    }

    public void init (HardwareMap hwmap) {
        mecDrive.init(hwmap, activeOpMode);
        lift.init(hwmap,activeOpMode);
        arm.init(hwmap, activeOpMode);
        gyro.init(hwmap);
        intake.init(hwmap, activeOpMode);
    }

    public void readSensors (){
        lift.currentPosition = lift.lift.getCurrentPosition();
        gyro.UpdateGyro(runtime);
    }

    public void mechanismLogic (Gamepad operatorGamepad){
        if (lift.currentPosition > lift.MIN_SAFE_HEIGHT_TICKS){
            arm.safeToRotate = true;
        } else {
            arm.safeToRotate = false;
        }

        if (arm.armPosition == Arm.ArmPositions.INTAKE && arm.armTargetPosition == Arm.ArmPositions.INTAKE){
            lift.safeToLower = true;
        } else {
            lift.safeToLower = false;
        }


    }

    public void operatorGamepadTranslation (Gamepad operatorGamepad, Gamepad lastOperatorGamepad){

        autoScoreButtonPressed = operatorGamepad.right_bumper;

        if (Math.abs(operatorGamepad.left_stick_y) > STICK_DEADZONE){
            targetLiftHeight = Lift.LiftHeights.MANUAL;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        } else if (Math.abs(operatorGamepad.left_stick_x) > STICK_DEADZONE) {
        }
        // Cone stack functions
        else if (operatorGamepad.right_stick_y < -STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.CONES_FIVE;
            targetArmPosition = Arm.ArmPositions.INTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.INTAKE;
        } else if (operatorGamepad.right_stick_y > STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.CONES_TWO;
            targetArmPosition = Arm.ArmPositions.INTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.INTAKE;
        } else if (operatorGamepad.right_stick_x > STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.CONES_THREE;
            targetArmPosition = Arm.ArmPositions.INTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.INTAKE;
        } else if (operatorGamepad.right_stick_x < -STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.CONES_FOUR;
            targetArmPosition = Arm.ArmPositions.INTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.INTAKE;
        }
        else if (operatorGamepad.y) {
            targetIntakeState = IntakeOuttake.IntakeStates.OUTTAKE;
        } else if (!operatorGamepad.y && lastOperatorGamepad.y) {
            targetIntakeState = IntakeOuttake.IntakeStates.READY_TO_INTAKE;
        }
        else if (operatorGamepad.x) {
            targetIntakeState = IntakeOuttake.IntakeStates.MANUAL_INTAKE;
        } else if (!operatorGamepad.x && targetIntakeState == IntakeOuttake.IntakeStates.MANUAL_INTAKE) {
            targetIntakeState = IntakeOuttake.IntakeStates.READY_TO_INTAKE;
        }
        else if (operatorGamepad.a) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_LOW;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        }
        else if (operatorGamepad.b && operatorGamepad.left_bumper) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_MEDIUM;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
            targetArmPosition = Arm.ArmPositions.FRONT_OUTTAKE;
        } else if (operatorGamepad.b && operatorGamepad.left_trigger > STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_MEDIUM;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
            targetArmPosition = Arm.ArmPositions.LEFT_OUTTAKE;
        } else if (operatorGamepad.b && operatorGamepad.right_trigger > STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_MEDIUM;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
            targetArmPosition = Arm.ArmPositions.RIGHT_OUTTAKE;
        } else if (operatorGamepad.b) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_MEDIUM;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        }
        else if (operatorGamepad.dpad_up) {
            targetArmPosition = Arm.ArmPositions.FRONT_OUTTAKE;
        } else if (operatorGamepad.dpad_down) {
            targetArmPosition = Arm.ArmPositions.INTAKE;
        } else if (operatorGamepad.dpad_left) {
            targetArmPosition = Arm.ArmPositions.LEFT_OUTTAKE;
        } else if (operatorGamepad.dpad_right) {
            //  targetArmPosition = Arm.ArmPositions.values()[Math.min(targetArmPosition.ordinal() +1,3)];
            targetArmPosition = Arm.ArmPositions.RIGHT_OUTTAKE;
        } else if (operatorGamepad.left_bumper && !lastOperatorGamepad.left_bumper) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_HIGH;
            targetArmPosition = Arm.ArmPositions.FRONT_OUTTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        }
        else if (operatorGamepad.right_bumper && autoScoreState == AutoScoreSteps.COMPLETE ){
            autoScoreState = AutoScoreSteps.OUTTAKE;
        //} else if (operatorGamepad.right_bumper && !lastOperatorGamepad.right_bumper  /*autoScoreState == AutoScoreSteps.WAIT_FOR_PRESS && !autoScoreButtonPressed*/) {
          //  autoScoreState = AutoScoreSteps.OUTTAKE;
        }
        else if (operatorGamepad.left_trigger > STICK_DEADZONE && lastOperatorGamepad.left_trigger < STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_HIGH;
            targetArmPosition = Arm.ArmPositions.LEFT_OUTTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        } else if (operatorGamepad.right_trigger > STICK_DEADZONE && lastOperatorGamepad.right_trigger < STICK_DEADZONE) {
            targetLiftHeight = Lift.LiftHeights.JUNCTION_HIGH;
            targetArmPosition = Arm.ArmPositions.RIGHT_OUTTAKE;
            targetIntakeState = IntakeOuttake.IntakeStates.HOLD_CONE;
        }
    }

    public void autoScore (){
        switch (autoScoreState){
            case INITIATE:
                liftHeightShift = true;
                autoScoreState = AutoScoreSteps.WAIT_FOR_PRESS;
                break;
            case WAIT_FOR_PRESS:
                if (lift.runToPosition != lift.lastRunToPosition){
                    autoScoreState = AutoScoreSteps.COMPLETE;
                }
                autoScoreWait++;
                break;
            case OUTTAKE:
                if (intake.outtakeComplete && targetIntakeState == IntakeOuttake.IntakeStates.OUTTAKE){
                    autoScoreState = AutoScoreSteps.RAISE_LIFT;
                }
                targetIntakeState = IntakeOuttake.IntakeStates.OUTTAKE;
                break;
            case RAISE_LIFT:
                if(lift.targetPosition - lift.currentPosition < lift.MAX_ALLOWED_DELTA_TICKS && !liftHeightShift){
                    autoScoreState = AutoScoreSteps.ROTATE_AND_LOWER;
                }
                liftHeightShift = false;
                autoScoreRaise++;
                break;
            case ROTATE_AND_LOWER:
                targetLiftHeight = Lift.LiftHeights.CONES_ONE;
                targetArmPosition = Arm.ArmPositions.INTAKE;
                targetIntakeState = IntakeOuttake.IntakeStates.INTAKE;
                autoScoreState = AutoScoreSteps.COMPLETE;
                autoScoreFinal++;
                break;
        }
    }

    public void executeFunctions (Gamepad driverGamepad, Gamepad operatorGamepad){
        mecDrive.driveModeSelection(driverGamepad,gyro.turnAngle,gyro.tiltAngle,gyro.tiltVelocity,gyro.tiltAccel);
        if (autoScoreState != AutoScoreSteps.COMPLETE){
            autoScore();
        }

        lift.runLift(targetLiftHeight, - operatorGamepad.left_stick_y, liftHeightShift);
        arm.rotateArm(targetArmPosition);
        intake.runIntakeOuttake(targetIntakeState);

        activeOpMode.telemetry.addData("target lift height", targetLiftHeight);
        activeOpMode.telemetry.addData("Auto Score State", autoScoreState);
    }

}
