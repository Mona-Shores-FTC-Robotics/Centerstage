package com.example.meepmeeptesting.Routes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

public class RobotCommands {

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
