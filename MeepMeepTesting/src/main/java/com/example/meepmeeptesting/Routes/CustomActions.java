package com.example.meepmeeptesting.Routes;

import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_CENTER;
import static com.example.meepmeeptesting.Constants.BLUE_BACKDROP_STAGING;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_PICKUP;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_PIXEL_WING;
import static com.example.meepmeeptesting.Constants.BLUE_NEUTRAL_STAGING;
import static com.example.meepmeeptesting.Constants.FACE_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_AUDIENCE;
import static com.example.meepmeeptesting.Constants.TANGENT_TOWARD_BACKSTAGE;
import static com.example.meepmeeptesting.MeepMeepTesting.roadRunnerDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class CustomActions {

    Action SetDeliverLocation()
    {
        SleepAction sleep = new SleepAction(1);
        return sleep;
    }


    Action AutoDriveToBackDrop(){
        Action autoDriveToBackdrop = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_STAGING)
                .lineToX(BLUE_BACKDROP_CENTER.position.x)
                .build();
        return autoDriveToBackdrop;
    }

    Action AutoDriveFromBackDrop(){
        Action autoDriveFromBackdrop = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_CENTER)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_STAGING, TANGENT_TOWARD_AUDIENCE)
                .build();
        return autoDriveFromBackdrop;
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

    public Action BackdropStagingToNeutralStaging(){
        Action backDropStagingToNeutralStaging = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_STAGING)
                .setReversed(true)
                .splineToLinearHeading(BLUE_NEUTRAL_STAGING, TANGENT_TOWARD_AUDIENCE)
                .build();
        return backDropStagingToNeutralStaging;
    }
    public Action NeutralStagingToBackdropStaging() {
        Action neutralStagingToBackdropStaging = roadRunnerDrive.actionBuilder(BLUE_NEUTRAL_STAGING)
                .splineToLinearHeading(BLUE_BACKDROP_STAGING, TANGENT_TOWARD_BACKSTAGE)
                .build();
        return neutralStagingToBackdropStaging;


    }
    public Action PickupPixels(){
        SequentialAction pickupPixels = new SequentialAction(
                new ParallelAction(
                        new CustomActions().TurnIntakeOn(),
                        new CustomActions().AutoDriveToNeutralStack()),
                new SleepAction(.1),
                new ParallelAction(
                        new CustomActions().TurnIntakeOff(),
                        new CustomActions().AutoDriveFromNeutralStack()));
        return pickupPixels;
    }

    private Action AutoDriveFromNeutralStack() {
        Action autoDriveFromNeutralStack = roadRunnerDrive.actionBuilder(BLUE_NEUTRAL_PIXEL_PICKUP)
                .splineToLinearHeading(BLUE_NEUTRAL_STAGING, TANGENT_TOWARD_BACKSTAGE)
                .build();
        return autoDriveFromNeutralStack;
    }

    public Action AutoDriveToNeutralStack() {
        Action autoDriveToNeutralStack = roadRunnerDrive.actionBuilder(BLUE_NEUTRAL_STAGING)
                .setReversed(true)
                .splineToLinearHeading(BLUE_NEUTRAL_PIXEL_PICKUP, TANGENT_TOWARD_AUDIENCE)
                .build();
        return autoDriveToNeutralStack;
    }

    public Action ScorePixelAction(){
        SequentialAction scorePixel = new SequentialAction(
                new ParallelAction(
                        new CustomActions().AutoDriveToBackDrop(),
                        new CustomActions().LiftLow(),
                        new CustomActions().RotateShoulderToBackdrop()),
                new SleepAction(.2),
                new CustomActions().CloseClaw(),
                new SleepAction(.2),
                new ParallelAction(
                        new CustomActions().AutoDriveFromBackDrop(),
                        new CustomActions().CloseClaw(),
                        new CustomActions().RotateShoulderToIntake()),
                new CustomActions().LiftHome()
            );
        return scorePixel;
    }


     Action ScoreFromStagingAndPickup4Pixels() {
         Action scoreAndPickup4Pixels = roadRunnerDrive.actionBuilder(BLUE_NEUTRAL_STAGING)
                 .stopAndAdd(new CustomActions().ScorePixelAction())
                 .stopAndAdd(new CustomActions().BackdropStagingToNeutralStaging())
                 .stopAndAdd(new CustomActions().PickupPixels())
                 .stopAndAdd(new CustomActions().NeutralStagingToBackdropStaging())
                 .stopAndAdd(new CustomActions().ScorePixelAction())
                 .stopAndAdd(new CustomActions().BackdropStagingToNeutralStaging())
                 .stopAndAdd(new CustomActions().PickupPixels())
                 .stopAndAdd(new CustomActions().NeutralStagingToBackdropStaging())
                 .stopAndAdd(new CustomActions().ScorePixelAction())
                 .build();
         return scoreAndPickup4Pixels;
     }
}
