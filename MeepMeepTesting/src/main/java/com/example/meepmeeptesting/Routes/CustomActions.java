package com.example.meepmeeptesting.Routes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

public class CustomActions {

    static Action waitForPartnerToGetOutOfWay()
    {
        SleepAction sleep = new SleepAction(4);
        return sleep;
    }

    static Action alignToLeftSideOfBackDropWithAprilTag()
    {
        SleepAction sleep = new SleepAction(1);
        return sleep;
    }

    static Action alignToRightSideOfBackDropWithAprilTag()
    {
        SleepAction sleep = new SleepAction(10);
        return sleep;
    }

    static Action alignToCenterSideOfBackDropWithAprilTag()
    {
        SleepAction sleep = new SleepAction(1);
        return sleep;
    }

    /** Methods for dropping pixels **/

    static Action extendSlide()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }

    static Action retractSlide()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }


    static Action dropPixelOnBackdrop()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }

    static Action pickupPixel()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }


}
