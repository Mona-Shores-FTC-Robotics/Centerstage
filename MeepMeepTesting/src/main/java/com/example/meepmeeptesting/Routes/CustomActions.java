package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_PICKUP;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_WING;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_STAGING;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.PoseToVector;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_LEFT;
import static com.example.meepmeeptesting.Constants.RED_BACKDROP_RIGHT;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.MeepMeepTesting.roadRunnerDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.meepmeeptesting.MeepMeepTesting;

public class CustomActions {

    Action SetDeliverLocation()
    {
        SleepAction sleep = new SleepAction(1);
        return sleep;
    }

    Action RotateShoulderToBackdrop(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action RotateShoulderToIntake(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action LiftHome(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action LiftLow(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action LiftMid(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action LiftHigh(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action OpenClaw(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action CloseClaw(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action TurnIntakeOn(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

    Action TurnIntakeOff(){
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    };

}
