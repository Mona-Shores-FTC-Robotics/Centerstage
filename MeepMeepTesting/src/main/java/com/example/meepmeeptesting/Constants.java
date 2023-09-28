// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Constants {

  //private static final RobotType ROBOT = RobotType.ROBOT_2023;
  private static final RobotType ROBOT = RobotType.ROBOT_CHASSIS;

  public enum RobotType {
        ROBOT_2023,
        ROBOT_CHASSIS,
        ROBOT_VISION
  }

  public static RobotType getRobot() {
     if (ROBOT == RobotType.ROBOT_2023) {
        return RobotType.ROBOT_2023;
      } else if (ROBOT == RobotType.ROBOT_VISION)
     {
         return RobotType.ROBOT_VISION;
     }
     else {
        return RobotType.ROBOT_CHASSIS;
      }
  }

    public static double ROBOT_LENGTH = 18.0;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2;

    public static double HALF_FIELD = 72.0;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE/2;
    public static double QUARTER_TILE = TILE/4;
    public static double THREE_QUARTER_TILE = TILE*.75;

    public static double FACE_TOWARD_RED = Math.toRadians(0);
    public static double FACE_45_DEGREES = Math.toRadians(45);
    public static double FACE_TOWARD_BACKSTAGE = Math.toRadians(90);
    public static double FACE_135_DEGREES = Math.toRadians(135);
    public static double FACE_TOWARD_BLUE = Math.toRadians(180);
    public static double FACE_225_DEGREES = Math.toRadians(225);
    public static double FACE_TOWARD_FRONTSTAGE = Math.toRadians(270);
    public static double FACE_315_DEGREES = Math.toRadians(315);

    public static Pose2d BLUE_BACKDROP = new Pose2d(-TILE-HALF_TILE, 2*TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP = new Pose2d(TILE+HALF_TILE, 2*TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_LEFT_START_POSE = new Pose2d(-HALF_FIELD + HALF_ROBOT_LENGTH,HALF_TILE, FACE_TOWARD_RED);
    public static Pose2d BLUE_RIGHT_START_POSE = new Pose2d(-HALF_FIELD + HALF_ROBOT_LENGTH, -HALF_TILE-TILE, FACE_TOWARD_RED);
    public static Pose2d RED_RIGHT_START_POSE = new Pose2d(HALF_FIELD - HALF_ROBOT_LENGTH, HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_LEFT_START_POSE = new Pose2d(+HALF_FIELD - HALF_ROBOT_LENGTH,-HALF_TILE - TILE, FACE_TOWARD_BLUE);

    public static Pose2d RED_BACKSTAGE_SPIKE_R = new Pose2d(TILE+HALF_TILE,TILE-QUARTER_TILE, FACE_135_DEGREES);
    public static Pose2d RED_BACKSTAGE_SPIKE_C = new Pose2d(TILE+HALF_TILE,HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_SPIKE_L = new Pose2d(TILE+HALF_TILE, TILE-THREE_QUARTER_TILE, FACE_225_DEGREES);

    public static Pose2d BLUE_BACKSTAGE_SPIKE_L = new Pose2d(-TILE-HALF_TILE,TILE-QUARTER_TILE, FACE_45_DEGREES);
    public static Pose2d BLUE_BACKSTAGE_SPIKE_C = new Pose2d(-TILE+HALF_TILE,HALF_TILE, FACE_TOWARD_RED);
    public static Pose2d BLUE_BACKSTAGE_SPIKE_R = new Pose2d(-TILE-HALF_TILE, TILE-THREE_QUARTER_TILE, FACE_315_DEGREES);

    public static Pose2d RED_AUDIENCE_SPIKE_R = new Pose2d(TILE+HALF_TILE,-TILE-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_SPIKE_C = new Pose2d(TILE+HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_SPIKE_L = new Pose2d(TILE+HALF_TILE, -TILE-THREE_QUARTER_TILE, FACE_TOWARD_BLUE);

    public static Pose2d BLUE_AUDIENCE_SPIKE_L =  new Pose2d(-TILE-HALF_TILE,-TILE-QUARTER_TILE, FACE_TOWARD_RED);
    public static Pose2d BLUE_AUDIENCE_SPIKE_C = new Pose2d(-TILE-HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_RED);
    public static Pose2d BLUE_AUDIENCE_SPIKE_R = new Pose2d(-TILE-HALF_TILE, -TILE-THREE_QUARTER_TILE, FACE_TOWARD_RED);


    public static Vector2d BLUE_BACKSTAGE_PARK = new Vector2d(-TILE*2-HALF_TILE, 2*TILE);
    public static Vector2d RED_BACKSTAGE_PARK = new Vector2d(TILE*2+HALF_TILE, 2*TILE);

    public static Pose2d RED_STAGEDOOR_ENTRANCE = new Pose2d(HALF_TILE,-HALF_TILE-TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_STAGEDOOR_ENTRANCE =new Pose2d(-HALF_TILE,-HALF_TILE-TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_STAGEDOOR_EXIT = new Pose2d(HALF_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_STAGEDOOR_EXIT = new Pose2d(-HALF_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Vector2d RED_NEUTRAL_PIXEL_STAGEDOOR = new Vector2d(HALF_TILE,-TILE*-QUARTER_TILE);
    public static Vector2d RED_NEUTRAL_PIXEL_CENTERSPIKE = new Vector2d(TILE,-TILE*2-QUARTER_TILE);
    public static Vector2d RED_NEUTRAL_PIXEL_WING = new Vector2d(TILE+HALF_TILE,-TILE*2-QUARTER_TILE);

    public static Vector2d BLUE_NEUTRAL_PIXEL_STAGEDOOR = new Vector2d(-HALF_TILE,-TILE*-QUARTER_TILE);
    public static Vector2d BLUE_NEUTRAL_PIXEL_CENTERSPIKE = new Vector2d(-TILE,-TILE*2-QUARTER_TILE);
    public static Vector2d BLUE_NEUTRAL_PIXEL_WING = new Vector2d(-TILE+HALF_TILE,-TILE*2-QUARTER_TILE);

    public static Pose2d RED_SAFE_STRAFE = new Pose2d(TILE+HALF_TILE,-TILE*2-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d BLUE_SAFE_STRAFE = new Pose2d(-TILE-HALF_TILE,-TILE*2-QUARTER_TILE, FACE_TOWARD_RED);

    public static Pose2d RED_THROUGH_DOOR = new Pose2d(HALF_TILE,TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_THROUGH_DOOR =new Pose2d(-HALF_TILE,TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);


    public static Pose2d changeToMeepMeepCoordinates(Pose2d pose){

        return new Pose2d(-pose.position.y, pose.position.x, Math.toRadians(pose.heading.real+pose.heading.imag+180));

    }

}