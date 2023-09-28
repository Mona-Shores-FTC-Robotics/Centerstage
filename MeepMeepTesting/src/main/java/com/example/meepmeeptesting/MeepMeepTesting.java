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

    public static RoadRunnerBotEntity blueBackstageBot;
    public static RoadRunnerBotEntity redAudienceBot;
    public static RoadRunnerBotEntity blueAudienceBot;
    public static RoadRunnerBotEntity redBackstageBot;

    enum teamPropLocation {LEFT, CENTER, RIGHT}
    public static teamPropLocation teamPropLocationFinal = teamPropLocation.RIGHT;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MakeRobots(meepMeep);

        if (teamPropLocationFinal == teamPropLocation.LEFT)
        {
            teamPropLeftRoute(blueBackstageBot, redBackstageBot, blueAudienceBot, redAudienceBot);
        }

        if (teamPropLocationFinal == teamPropLocation.CENTER)
        {
            teamPropCenterRoute(blueBackstageBot, redBackstageBot, blueAudienceBot, redAudienceBot);
        }

        if (teamPropLocationFinal == teamPropLocation.RIGHT)
        {
            teamPropRightRoute(blueBackstageBot, redBackstageBot, blueAudienceBot, redAudienceBot);
        }

        String filePath = "Centerstage.png";
        System.out.println(new File(".").getAbsolutePath());
        Image img = null;
        try { img = ImageIO.read(new File(filePath)); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBackstageBot)
                .addEntity(blueBackstageBot)
                .addEntity(redAudienceBot)
                .addEntity(blueAudienceBot)
                .start();
    }

    private static void MakeRobots( MeepMeep meepMeep ) {
        blueBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        blueAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        redAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        redBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();
    }

    static void teamPropCenterRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
{
    blueBackstageBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
            .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, FACE_TOWARD_RED)
            .setReversed(true)
            .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .strafeTo(BLUE_BACKSTAGE_PARK)
            .turnTo(FACE_TOWARD_FRONTSTAGE)
            .build());

    redBackstageBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
            .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, FACE_TOWARD_BLUE)
            .setReversed(true)
            .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .strafeTo(RED_BACKSTAGE_PARK)
            .turnTo(FACE_TOWARD_FRONTSTAGE)
            .build());

    redAudienceBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
            .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, FACE_TOWARD_BLUE)
            .splineToLinearHeading(RED_SAFE_STRAFE , FACE_TOWARD_BLUE)
            .splineToLinearHeading(new Pose2d(RED_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_BLUE) , FACE_TOWARD_BLUE)
            .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(RED_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .build());

    blueAudienceBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
            .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, FACE_TOWARD_RED)
            .splineToLinearHeading(BLUE_SAFE_STRAFE , FACE_TOWARD_RED)
            .splineToLinearHeading(new Pose2d(BLUE_NEUTRAL_PIXEL_CENTERSPIKE, FACE_TOWARD_RED), FACE_TOWARD_RED)
            .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(BLUE_THROUGH_DOOR, FACE_TOWARD_BACKSTAGE)
            .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
            .build());
}


    static void teamPropLeftRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
    {
        blueBackstageBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, FACE_TOWARD_RED)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(BLUE_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .lineToY(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redBackstageBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, FACE_TOWARD_BLUE)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redAudienceBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(new Pose2d(RED_AUDIENCE_SPIKE_L.position.x, RED_AUDIENCE_SPIKE_L.position.y, FACE_225_DEGREES), FACE_225_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RED_STAGEDOOR_ENTRANCE.position.x, RED_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());

        blueAudienceBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(new Pose2d(BLUE_AUDIENCE_SPIKE_L.position.x, BLUE_AUDIENCE_SPIKE_L.position.y, FACE_45_DEGREES), FACE_45_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(BLUE_STAGEDOOR_ENTRANCE.position.x, BLUE_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(BLUE_THROUGH_DOOR.position.y)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());
    }

    static void teamPropRightRoute(RoadRunnerBotEntity blueLeftBot, RoadRunnerBotEntity redRightBot, RoadRunnerBotEntity blueRightBot, RoadRunnerBotEntity redLeftBot )
    {
        blueBackstageBot.runAction(blueLeftBot.getDrive().actionBuilder(BLUE_LEFT_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, FACE_TOWARD_RED)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .strafeTo(BLUE_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());

        redBackstageBot.runAction(redRightBot.getDrive().actionBuilder(RED_RIGHT_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, FACE_TOWARD_BLUE)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(RED_STAGEDOOR_EXIT, FACE_TOWARD_BACKSTAGE)
                .lineToY(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                         .strafeTo(RED_BACKSTAGE_PARK)
                .turnTo(FACE_TOWARD_FRONTSTAGE)
                .build());


        redAudienceBot.runAction(redLeftBot.getDrive().actionBuilder(RED_LEFT_START_POSE)
                .splineToLinearHeading(new Pose2d(RED_AUDIENCE_SPIKE_R.position.x, RED_AUDIENCE_SPIKE_R.position.y, FACE_135_DEGREES), FACE_TOWARD_BACKSTAGE)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RED_STAGEDOOR_ENTRANCE.position.x, RED_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .turnTo(FACE_TOWARD_BACKSTAGE)
                .lineToY(RED_THROUGH_DOOR.position.y)
                .splineToLinearHeading(RED_BACKDROP, FACE_TOWARD_BACKSTAGE)
                .build());

        blueAudienceBot.runAction(blueRightBot.getDrive().actionBuilder(BLUE_RIGHT_START_POSE)
                .splineToLinearHeading(new Pose2d(BLUE_AUDIENCE_SPIKE_R.position.x, BLUE_AUDIENCE_SPIKE_R.position.y, FACE_315_DEGREES), FACE_315_DEGREES)
                .stopAndAdd(dropPixel())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(BLUE_STAGEDOOR_ENTRANCE.position.x, BLUE_STAGEDOOR_ENTRANCE.position.y, FACE_TOWARD_BACKSTAGE), FACE_TOWARD_RED)
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

