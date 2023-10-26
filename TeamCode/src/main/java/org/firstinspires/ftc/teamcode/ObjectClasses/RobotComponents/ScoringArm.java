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

    public ScoringArm() {

    }

    public void init() {
        endEffector.init();
        liftSlide.init();
        shoulder.init();
    }

    public SequentialAction grabAndScorePixelOnBackdropLow = new SequentialAction(
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

    public SequentialAction grabAndScorePixelOnBackdropMid = new SequentialAction(
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

    public SequentialAction grabAndScorePixelOnBackdropHigh = new SequentialAction(
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
