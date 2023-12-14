package com.example.meepmeeptesting;
import com.example.meepmeeptesting.Routes.RoutesSpikeBackdropPark;
import com.example.meepmeeptesting.Routes.RoutesSpikeOnly;
import com.example.meepmeeptesting.Routes.RoutesSpikeStraightUpTheMiddle;
import com.example.meepmeeptesting.Routes.RoutesSuper;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import static com.example.meepmeeptesting.MeepMeepRobots.*;

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

    public static TeamPropLocation teamPropLocation = TeamPropLocation.CENTER;

    public static RoutesToRun routesToRunSelection = RoutesToRun.SUPER;

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = false;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = true;
    public static boolean SHOW_RED_AUDIENCE_BOT = false;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

    public static DriveShim roadRunnerDrive;
    public enum TeamPropLocation {LEFT, CENTER, RIGHT, ALL}

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum SideOfField {BACKSTAGE, AUDIENCE}
    public enum LiftStates {
        AUTO_LOW, AUTO_MID, AUTO_HIGH, MAX, HIGH, MID, LOW, SAFE, HOME, ZERO, MANUAL}

    public enum GripperStates {
        REST (0.5),
        CLOSED (.55),
        OPEN (.47),
        ONE_PIXEL_RELEASE_POSITION (.490);
        public double position;
        GripperStates(double pos) {
            this.position = pos;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public enum ShoulderStates {
        REST (.5),
        INTAKE (0.54),
        HALFWAY(.4),
        BACKDROP (.2),
        STARTING_POSITION (.565);

        public double position;
        ShoulderStates(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }
    public enum PixelPusherStates {
        NOT_PUSHING,
        PUSHING;
    }

    enum RoutesToRun {SPIKE_ONLY, SPIKE_BACKDROP_PARK, SPIKE_BACKDROP_PARK_IMPROVED, SPIKE_PICKUP1_BACKDROP_PARK, SPIKE_STRAIGHT, SUPER}

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        roadRunnerDrive = MeepMeepRobots.roadRunnerBot.getDrive();

        //Set the routes that will be run
        if (routesToRunSelection == RoutesToRun.SPIKE_BACKDROP_PARK) {

            RoutesSpikeBackdropPark.BuildRoutes();

            if (teamPropLocation == TeamPropLocation.LEFT) RoutesSpikeBackdropPark.setTeamPropLeftRoutes();
            if (teamPropLocation == TeamPropLocation.CENTER) RoutesSpikeBackdropPark.setTeamPropCenterRoutes();
            if (teamPropLocation == TeamPropLocation.RIGHT) RoutesSpikeBackdropPark.setTeamPropRightRoutes();
            if (teamPropLocation == TeamPropLocation.ALL) RoutesSpikeBackdropPark.setTeamPropAllRoutes();

        }
        else if (routesToRunSelection == RoutesToRun.SPIKE_ONLY) {

            RoutesSpikeOnly.BuildRoutes();

            if (teamPropLocation == TeamPropLocation.LEFT) RoutesSpikeOnly.setTeamPropLeftRoutes();
            if (teamPropLocation == TeamPropLocation.CENTER) RoutesSpikeOnly.setTeamPropCenterRoutes();
            if (teamPropLocation == TeamPropLocation.RIGHT) RoutesSpikeOnly.setTeamPropRightRoutes();
            if (teamPropLocation == TeamPropLocation.ALL) RoutesSpikeOnly.setTeamPropAllRoutes();

        }

        else if (routesToRunSelection == RoutesToRun.SPIKE_STRAIGHT) {

            RoutesSpikeStraightUpTheMiddle.BuildRoutes();

            if (teamPropLocation == TeamPropLocation.LEFT) RoutesSpikeStraightUpTheMiddle.setTeamPropLeftRoutes();
            if (teamPropLocation == TeamPropLocation.CENTER) RoutesSpikeStraightUpTheMiddle.setTeamPropCenterRoutes();
            if (teamPropLocation == TeamPropLocation.RIGHT) RoutesSpikeStraightUpTheMiddle.setTeamPropRightRoutes();
            if (teamPropLocation == TeamPropLocation.ALL) RoutesSpikeStraightUpTheMiddle.setTeamPropAllRoutes();
        } else if (routesToRunSelection == RoutesToRun.SUPER) {

            RoutesSuper.BuildRoutes();

            if (teamPropLocation == TeamPropLocation.LEFT)
                RoutesSuper.setTeamPropLeftRoutes();
            if (teamPropLocation == TeamPropLocation.CENTER)
                RoutesSuper.setTeamPropCenterRoutes();
            if (teamPropLocation == TeamPropLocation.RIGHT)
                RoutesSuper.setTeamPropRightRoutes();
            if (teamPropLocation == TeamPropLocation.ALL)
                RoutesSuper.setTeamPropAllRoutes();
        }

        addRobotsToField(meepMeep);

    }

    private static void addRobotsToField(MeepMeep meepMeep_local) {



        if (teamPropLocation != TeamPropLocation.ALL) {
            if (SHOW_BLUE_AUDIENCE_BOT) meepMeep_local.addEntity(blueAudienceBot);
            if (SHOW_BLUE_BACKSTAGE_BOT) meepMeep_local.addEntity(blueBackstageBot);
            if (SHOW_RED_AUDIENCE_BOT) meepMeep_local.addEntity(redAudienceBot);
            if (SHOW_RED_BACKSTAGE_BOT) meepMeep_local.addEntity(redBackstageBot);
        }

        if (teamPropLocation == TeamPropLocation.ALL)
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

