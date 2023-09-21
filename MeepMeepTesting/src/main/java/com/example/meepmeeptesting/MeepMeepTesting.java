package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static com.example.meepmeeptesting.Constants.*;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {

    public static RoadRunnerBotEntity blueLeftBot;
    public static RoadRunnerBotEntity redLeftBot;
    public static RoadRunnerBotEntity blueRightBot;
    public static RoadRunnerBotEntity redRightBot;

    enum teamPropLocation {LEFT, CENTER, RIGHT}
    public static teamPropLocation teamPropLocationFinal = teamPropLocation.LEFT;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MakeRobots(meepMeep);

        if (teamPropLocationFinal == teamPropLocation.LEFT)
        {
            teamPropLeftRoute(blueLeftBot, redRightBot, blueRightBot, redLeftBot);
        }

        if (teamPropLocationFinal == teamPropLocation.CENTER)
        {
            teamPropCenterRoute(blueLeftBot, redRightBot, blueRightBot, redLeftBot);
        }

        if (teamPropLocationFinal == teamPropLocation.RIGHT)
        {
            teamPropRightRoute(blueLeftBot, redRightBot, blueRightBot, redLeftBot);
        }

        String filePath = "Centerstage.png";
        System.out.println(new File(".").getAbsolutePath());
        Image img = null;
        try { img = ImageIO.read(new File(filePath)); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setBackgroundAlpha(0.95f)
                .addEntity(redRightBot)
                .addEntity(blueLeftBot)
                .addEntity(redLeftBot)
                .addEntity(blueRightBot)
                .start();

    }

    private static void MakeRobots( MeepMeep meepMeep ) {
        blueLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        redLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        redRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();
    }

    static void teamPropCenterRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
{
    blueLeftBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
            .splineToLinearHeading(BLUE_LEFT_SPIKE_LOCATION, FACE_TOWARD_RED)
            .turnTo(FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .strafeTo(BLUE_BACKSTAGE_PARK)
            .turnTo(FACE_TOWARD_FRONTSTAGE)
            .build());

    redRightBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
            .splineToLinearHeading(RED_RIGHT_SPIKE_LOCATION, Math.toRadians(180))
            .turnTo(FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .strafeTo(RED_BACKSTAGE_PARK)
            .turnTo(FACE_TOWARD_FRONTSTAGE)
            .build());

    redLeftBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
            .splineToLinearHeading( RED_LEFT_SPIKE_LOCATION, FACE_TOWARD_BLUE)
            .splineToLinearHeading(RED_SAFE_STRAFE , FACE_TOWARD_BLUE)
            .splineToLinearHeading(new Pose2d(RED_NEUTRAL_PIXEL_2, FACE_TOWARD_BLUE) , FACE_TOWARD_BLUE)
            .splineToLinearHeading(RED_STAGEDOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .build());

    blueRightBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
            .splineToLinearHeading(BLUE_RIGHT_SPIKE_LOCATION, FACE_TOWARD_RED)
            .splineToLinearHeading(BLUE_SAFE_STRAFE , FACE_TOWARD_RED)
            .splineToLinearHeading(new Pose2d(BLUE_NEUTRAL_PIXEL_2, FACE_TOWARD_RED), FACE_TOWARD_RED)
            .splineToLinearHeading(BLUE_STAGEDOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .build());
}


    static void teamPropLeftRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
    {
        blueLeftBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_LEFT_SPIKE_LOCATION, FACE_TOWARD_RED)
                .stopAndAdd(dropPixel())
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redRightBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_RIGHT_SPIKE_LOCATION, FACE_TOWARD_BLUE)
                .stopAndAdd(dropPixel())
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redLeftBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(new Pose2d(RED_SPIKE_3.position.x, RED_SPIKE_3.position.y, FACE_225_DEGREES), FACE_225_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RED_STAGEDOOR.position.x, RED_STAGEDOOR.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());

        blueRightBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(new Pose2d(BLUE_SPIKE_1.position.x, BLUE_SPIKE_1.position.y, FACE_45_DEGREES), FACE_45_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(BLUE_STAGEDOOR.position.x, BLUE_STAGEDOOR.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());
    }

    static void teamPropRightRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
    {
        blueLeftBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_LEFT_SPIKE_LOCATION, FACE_TOWARD_RED)
                .stopAndAdd(dropPixel())
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redRightBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_RIGHT_SPIKE_LOCATION, FACE_TOWARD_BLUE)
                .stopAndAdd(dropPixel())
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());


        redLeftBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(new Pose2d(RED_SPIKE_1.position.x, RED_SPIKE_1.position.y, FACE_135_DEGREES), FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RED_STAGEDOOR.position.x, RED_STAGEDOOR.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());

        blueRightBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(new Pose2d(BLUE_SPIKE_3.position.x, BLUE_SPIKE_3.position.y, FACE_315_DEGREES), FACE_315_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(BLUE_STAGEDOOR.position.x, BLUE_STAGEDOOR.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_RED)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());
    }

    static Action dropPixel()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }

}

