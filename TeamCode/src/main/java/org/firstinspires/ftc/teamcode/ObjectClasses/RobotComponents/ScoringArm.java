package org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.EndEffector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.LiftSlide;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotComponents.ArmComponents.Shoulder;

public final class ScoringArm{

    EndEffector endEffector = new EndEffector();
    LiftSlide liftSlide = new LiftSlide();
    Shoulder shoulder = new Shoulder();

    public SequentialAction grabAndScorePixelOnBackdropLow;
    public SequentialAction grabAndScorePixelOnBackdropMid;
    public SequentialAction grabAndScorePixelOnBackdropHigh;

    public ScoringArm() {

    }

    public void init() {
        endEffector.init();
        liftSlide.init();
        shoulder.init();

        grabAndScorePixelOnBackdropLow = new SequentialAction(
                liftSlide.liftToLowHeight(),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                endEffector.closeEndEffector(),
                liftSlide.liftToLowHeight(),
                shoulder.rotate(Shoulder.ShoulderStates.BACKDROP),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                liftSlide.liftToLowHeight()
        );

        grabAndScorePixelOnBackdropMid = new SequentialAction(
                liftSlide.liftToLowHeight(),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                endEffector.closeEndEffector(),
                liftSlide.liftToMidHeight(),
                shoulder.rotate(Shoulder.ShoulderStates.BACKDROP),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                liftSlide.liftToLowHeight()
        );

        grabAndScorePixelOnBackdropHigh = new SequentialAction(
                liftSlide.liftToLowHeight(),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                endEffector.closeEndEffector(),
                liftSlide.liftToHighHeight(),
                shoulder.rotate(Shoulder.ShoulderStates.BACKDROP),
                endEffector.openEndEffector(),
                shoulder.rotate(Shoulder.ShoulderStates.INTAKE),
                liftSlide.liftToLowHeight()
        );
    }
}
