package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.example.meepmeeptesting.Routes.RoutesSpikeBackdropPark;
import com.example.meepmeeptesting.Routes.RoutesSpikeOnly;
import com.example.meepmeeptesting.Routes.RoutesSpikePickup1BackdropPark;
import com.example.meepmeeptesting.Routes.RoutesSpikePickup1BackdropPickup2BackdropPark;
import com.example.meepmeeptesting.Routes.RoutesSpikeStraightUpTheMiddle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import static com.example.meepmeeptesting.MeepMeepRobots.*;

import java.awt.Color;
import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    /**
     * SET TO PICK WHICH ROUTES TO RUN:
     * Prop Location: LEFT, RIGHT, OR CENTER
     * Routes:
     *      SPIKE_ONLY,
     *      SPIKE_BACKDROP_PARK,
     *      SPIKE_PICKUP1_BACKDROP_PARK,
     *      SPIKE_PICKUP1_BACKDROP_PICKUP2_BACKDROP_PARK
     **/

    public static teamPropLocation teamPropLocationFinal = teamPropLocation.LEFT;
    public static routesToRun routesToRunSelection = routesToRun.SPIKE_STRAIGHT;

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = true;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = false;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = false;

    public static DriveShim roadRunnerDrive;
    enum teamPropLocation {LEFT, CENTER, RIGHT, ALL}
    enum routesToRun {SPIKE_ONLY, SPIKE_BACKDROP_PARK, SPIKE_PICKUP1_BACKDROP_PARK, SPIKE_PICKUP1_BACKDROP_PICKUP2_BACKDROP_PARK, SPIKE_STRAIGHT}

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);



        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        roadRunnerDrive = MeepMeepRobots.roadRunnerBot.getDrive();

        //Set the routes that will be run
        if (routesToRunSelection == routesToRun.SPIKE_BACKDROP_PARK) {

            RoutesSpikeBackdropPark.BuildRoutes();

            if (teamPropLocationFinal == teamPropLocation.LEFT) RoutesSpikeBackdropPark.setTeamPropLeftRoutes();
            if (teamPropLocationFinal == teamPropLocation.CENTER) RoutesSpikeBackdropPark.setTeamPropCenterRoutes();
            if (teamPropLocationFinal == teamPropLocation.RIGHT) RoutesSpikeBackdropPark.setTeamPropRightRoutes();
            if (teamPropLocationFinal == teamPropLocation.ALL) RoutesSpikeBackdropPark.setTeamPropAllRoutes();
        } else if (routesToRunSelection == routesToRun.SPIKE_PICKUP1_BACKDROP_PARK) {

            RoutesSpikePickup1BackdropPark.BuildRoutes();

            if (teamPropLocationFinal == teamPropLocation.LEFT) RoutesSpikePickup1BackdropPark.setTeamPropLeftRoutes();
            if (teamPropLocationFinal == teamPropLocation.CENTER) RoutesSpikePickup1BackdropPark.setTeamPropCenterRoutes();
            if (teamPropLocationFinal == teamPropLocation.RIGHT) RoutesSpikePickup1BackdropPark.setTeamPropRightRoutes();
            if (teamPropLocationFinal == teamPropLocation.ALL) RoutesSpikePickup1BackdropPark.setTeamPropAllRoutes();

        } else if (routesToRunSelection == routesToRun.SPIKE_ONLY) {

            RoutesSpikeOnly.BuildRoutes();

            if (teamPropLocationFinal == teamPropLocation.LEFT) RoutesSpikeOnly.setTeamPropLeftRoutes();
            if (teamPropLocationFinal == teamPropLocation.CENTER) RoutesSpikeOnly.setTeamPropCenterRoutes();
            if (teamPropLocationFinal == teamPropLocation.RIGHT) RoutesSpikeOnly.setTeamPropRightRoutes();
            if (teamPropLocationFinal == teamPropLocation.ALL) RoutesSpikeOnly.setTeamPropAllRoutes();

        } else if (routesToRunSelection == routesToRun.SPIKE_PICKUP1_BACKDROP_PICKUP2_BACKDROP_PARK) {

            RoutesSpikePickup1BackdropPickup2BackdropPark.BuildRoutes();

            if (teamPropLocationFinal == teamPropLocation.LEFT) RoutesSpikePickup1BackdropPickup2BackdropPark.setTeamPropLeftRoutes();
            if (teamPropLocationFinal == teamPropLocation.CENTER) RoutesSpikePickup1BackdropPickup2BackdropPark.setTeamPropCenterRoutes();
            if (teamPropLocationFinal == teamPropLocation.RIGHT) RoutesSpikePickup1BackdropPickup2BackdropPark.setTeamPropRightRoutes();
            if (teamPropLocationFinal == teamPropLocation.ALL) RoutesSpikePickup1BackdropPickup2BackdropPark.setTeamPropAllRoutes();
        }

        else if (routesToRunSelection == routesToRun.SPIKE_STRAIGHT) {

            RoutesSpikeStraightUpTheMiddle.BuildRoutes();

            if (teamPropLocationFinal == teamPropLocation.LEFT) RoutesSpikeStraightUpTheMiddle.setTeamPropLeftRoutes();
            if (teamPropLocationFinal == teamPropLocation.CENTER) RoutesSpikeStraightUpTheMiddle.setTeamPropCenterRoutes();
            if (teamPropLocationFinal == teamPropLocation.RIGHT) RoutesSpikeStraightUpTheMiddle.setTeamPropRightRoutes();
            if (teamPropLocationFinal == teamPropLocation.ALL) RoutesSpikeStraightUpTheMiddle.setTeamPropAllRoutes();
        }

        addRobotsToField(meepMeep);

    }

    private static void addRobotsToField(MeepMeep meepMeep_local) {



        if (teamPropLocationFinal != teamPropLocation.ALL) {
            if (SHOW_BLUE_AUDIENCE_BOT) meepMeep_local.addEntity(blueAudienceBot);
            if (SHOW_BLUE_BACKSTAGE_BOT) meepMeep_local.addEntity(blueBackstageBot);
            if (SHOW_RED_AUDIENCE_BOT) meepMeep_local.addEntity(redAudienceBot);
            if (SHOW_RED_BACKSTAGE_BOT) meepMeep_local.addEntity(redBackstageBot);
        }

        if (teamPropLocationFinal == teamPropLocation.ALL)
        {
            if (SHOW_BLUE_BACKSTAGE_BOT){
                meepMeep_local.addEntity(blueBackstageBot);
                meepMeep_local.addEntity(blueBackstageBotLeft);
                meepMeep_local.addEntity(blueBackstageBotRight);
            }

            if (SHOW_BLUE_AUDIENCE_BOT){
                meepMeep_local.addEntity(blueAudienceBot);
                meepMeep_local.addEntity(blueAudienceBotLeft);
                meepMeep_local.addEntity(blueAudienceBotRight);
            }

            if (SHOW_RED_BACKSTAGE_BOT){
                meepMeep_local.addEntity(redBackstageBot);
                meepMeep_local.addEntity(redBackstageBotLeft);
                meepMeep_local.addEntity(redBackstageBotRight);
            }

            if (SHOW_RED_AUDIENCE_BOT){
                meepMeep_local.addEntity(redAudienceBot);
                meepMeep_local.addEntity(redAudienceBotLeft);
                meepMeep_local.addEntity(redAudienceBotRight);
            }

        }


        String filePath = "CenterstageRotated.png";
        System.out.println(new File(".").getAbsolutePath());
        Image img = null;
        try {
            img = ImageIO.read(new File(filePath));
        } catch (IOException e) {
        }


        meepMeep_local.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(.95f)
                .start();

    }

}

