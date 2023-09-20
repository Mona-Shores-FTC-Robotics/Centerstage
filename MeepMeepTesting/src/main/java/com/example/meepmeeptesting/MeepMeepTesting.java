package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        RoadRunnerBotEntity blueRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        RoadRunnerBotEntity redLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        RoadRunnerBotEntity redRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        double WIDTH_OF_ROBOT_INCHES = 18.0;
        double LENGTH_OF_ROBOT_INCHES = 18.0;

        double HALF_WIDTH_FIELD_INCHES = 72.0;
        double HALF_LENGTH_FIELD_INCHES = 72.0;

        double WIDTH_OF_TILE_INCHES = 23.5;
        double LENGTH_OF_TILE_INCHES = 23.5;

        Pose2d RedRightSpikeLocation=new Pose2d(WIDTH_OF_TILE_INCHES+WIDTH_OF_TILE_INCHES/2,LENGTH_OF_TILE_INCHES/2,Math.toRadians(180));

        Pose2d RedRightStartPose = new Pose2d(HALF_WIDTH_FIELD_INCHES-(WIDTH_OF_ROBOT_INCHES/2), LENGTH_OF_TILE_INCHES/2, Math.toRadians(180));

        Pose2d BlueLeftStartPose = new Pose2d(-HALF_WIDTH_FIELD_INCHES+(WIDTH_OF_ROBOT_INCHES/2),LENGTH_OF_TILE_INCHES/2,Math.toRadians(0));

        Pose2d BlueRightStartPose = new Pose2d(-HALF_WIDTH_FIELD_INCHES+(WIDTH_OF_ROBOT_INCHES/2), (-LENGTH_OF_TILE_INCHES/2)-LENGTH_OF_TILE_INCHES, Math.toRadians(0));

        Pose2d RedLeftStartPose = new Pose2d(+HALF_WIDTH_FIELD_INCHES-(WIDTH_OF_ROBOT_INCHES/2),(-LENGTH_OF_TILE_INCHES/2)-LENGTH_OF_TILE_INCHES,Math.toRadians(180));

        Pose2d RedLeftSpikeLocation = new Pose2d(RedRightSpikeLocation.position.x, RedRightSpikeLocation.position.y-(WIDTH_OF_TILE_INCHES*2), Math.toRadians(180));

        redLeftBot.runAction(redLeftBot.getDrive().actionBuilder(RedLeftStartPose)
                .splineToLinearHeading( RedLeftSpikeLocation, Math.toRadians(180))
                .build());


        blueLeftBot.runAction(blueLeftBot.getDrive().actionBuilder(BlueLeftStartPose)
                .splineToLinearHeading(mirrorAcrossXAxis(RedRightSpikeLocation), Math.toRadians(0))
                .build());

        redRightBot.runAction(redRightBot.getDrive().actionBuilder(RedRightStartPose)
                        .splineToLinearHeading(RedRightSpikeLocation, Math.toRadians(180))
                .build());

       blueRightBot.runAction(blueRightBot.getDrive().actionBuilder(BlueRightStartPose)
               .splineToLinearHeading(mirrorAcrossXAxis(RedLeftSpikeLocation), Math.toRadians(0))
                .build());

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


    public static Pose2d mirrorAcrossXAxis(Pose2d input) {
        Pose2d output = new Pose2d(-input.position.x, input.position.y, Math.toRadians(0));
        return output;
    }
}
