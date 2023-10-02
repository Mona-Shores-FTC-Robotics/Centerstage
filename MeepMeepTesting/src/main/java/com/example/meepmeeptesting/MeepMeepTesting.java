package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static com.example.meepmeeptesting.Constants.*;
import static com.example.meepmeeptesting.Routes.*;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {

    public static RoadRunnerBotEntity blueBackstageBot;
    public static RoadRunnerBotEntity redAudienceBot;
    public static RoadRunnerBotEntity blueAudienceBot;
    public static RoadRunnerBotEntity redBackstageBot;

    public static RoadRunnerBotEntity roadRunnerBot;
    public static DriveShim roadRunnerDrive;

    enum teamPropLocation {LEFT, CENTER, RIGHT}
    public static teamPropLocation teamPropLocationFinal = teamPropLocation.CENTER;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MakeRobots(meepMeep);
        roadRunnerDrive = roadRunnerBot.getDrive();

        Routes.BuildTeamPropCenterRoutes();
        Routes.BuildTeamPropLeftRoutes();
        Routes.BuildTeamPropRightRoutes();

        if (teamPropLocationFinal == teamPropLocation.LEFT) teamPropLeftRoute();
        if (teamPropLocationFinal == teamPropLocation.CENTER) teamPropCenterRoute();
        if (teamPropLocationFinal == teamPropLocation.RIGHT) teamPropRightRoute();

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
                .setDimensions(16,16)
                .build();

        redBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        roadRunnerBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();
    }

    static void teamPropCenterRoute()
{
    blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
    redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
    redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
    blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
}

    static void teamPropLeftRoute()
    {
        blueBackstageBot.runAction(blueBackstageBotTeamPropLeftRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropLeftRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropLeftRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropLeftRoute);
    }

    static void teamPropRightRoute()
    {
        blueBackstageBot.runAction(blueBackstageBotTeamPropRightRoute);
        redBackstageBot.runAction(redBackstageBotTeamPropRightRoute);
        redAudienceBot.runAction(redAudienceBotTeamPropRightRoute);
        blueAudienceBot.runAction(blueAudienceBotTeamPropRightRoute);
    }

    static Action dropPixel()
    {
        SleepAction sleep = new SleepAction(.1);
        return sleep;
    }
}

