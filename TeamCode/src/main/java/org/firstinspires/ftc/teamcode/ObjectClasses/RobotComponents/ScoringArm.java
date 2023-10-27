package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.EndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.LiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.Shoulder;

public final class ScoringArm {

    public EndEffector endEffector = new EndEffector();
    public LiftSlide liftSlide = new LiftSlide();
    public Shoulder shoulder = new Shoulder();

    public ScoringArm() {

    }

    public void init() {
        endEffector.init();
        liftSlide.init();
        shoulder.init();
    }

    public Action makeGrabAndScorePixelOnBackdropMid() {
        Action test1 = new SequentialAction(
                endEffector.actuate(EndEffector.EndEffectorStates.OPEN),
                new SleepAction(1),
                endEffector.actuate(EndEffector.EndEffectorStates.CLOSED),
                new SleepAction(1),
                endEffector.actuate(EndEffector.EndEffectorStates.OPEN),
                new SleepAction(1),
                endEffector.actuate(EndEffector.EndEffectorStates.CLOSED));
        return test1;
    }


    public Action grabAndScorePixelOnBackdropLow() {
        Action test2 = new SequentialAction(
                liftSlide.liftToMidHeight(),
                liftSlide.liftToHighHeight(),
                liftSlide.liftToLowHeight(),
                liftSlide.liftToHighHeight()
        );
        return test2;
    }
}