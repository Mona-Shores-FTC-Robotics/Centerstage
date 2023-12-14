// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package org.firstinspires.ftc.teamcode.ObjectClasses.Constants;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldConstants {

    // TIME CONSTANTS

  public static double SCORE_DISTANCE = 7.8;
    public static double BACKSTAGE_ROBOT_WAIT_TIME = 2;
  public static double AUDIENCE_ROBOT_WAIT_TIME = 4.5;

    public static double END_GAME_TIME = 90;

    public static double ROBOT_LENGTH = 18.0;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH / 2;

    public static double HALF_FIELD = 72.0;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE / 2;
    public static double QUARTER_TILE = TILE / 4;
    public static double EIGHTH_TILE = TILE / 8;
    public static double THREE_QUARTER_TILE = TILE * .75;

    public static double FACE_TOWARD_BACKSTAGE = Math.toRadians(0);
    public static double FACE_30_DEGREES = Math.toRadians(30);
    public static double FACE_45_DEGREES = Math.toRadians(45);
    public static double FACE_75_DEGREES = Math.toRadians(75);
    public static double FACE_TOWARD_BLUE = Math.toRadians(90);
    public static double FACE_115_DEGREES = Math.toRadians(115);
    public static double FACE_135_DEGREES = Math.toRadians(135);
    public static double FACE_160_DEGREES = Math.toRadians(160);
    public static double FACE_TOWARD_AUDIENCE = Math.toRadians(180);
    public static double FACE_225_DEGREES = Math.toRadians(225);
    public static double FACE_TOWARD_RED = Math.toRadians(270);
    public static double FACE_315_DEGREES = Math.toRadians(315);
    public static double FACE_340_DEGREES = Math.toRadians(340);

    public static double TANGENT_TOWARD_BACKSTAGE = Math.toRadians(0);
    public static double TANGENT_45_DEGREES = Math.toRadians(45);
    public static double TANGENT_75_DEGREES = Math.toRadians(75);
    public static double TANGENT_TOWARD_BLUE = Math.toRadians(90);
    public static double TANGENT_135_DEGREES = Math.toRadians(135);
    public static double TANGENT_TOWARD_AUDIENCE = Math.toRadians(180);
    public static double TANGENT_225_DEGREES = Math.toRadians(225);
    public static double TANGENT_TOWARD_RED = Math.toRadians(270);
    public static double TANGENT_315_DEGREES = Math.toRadians(315);

    public static double DISTANCE_TO_PIXEL_STACK_FROM_STAGING=3;
    public static double DISTANCE_TO_STAGE_DOOR_PIXEL_STACK_FROM_STAGING=3;
    /////////////////
    // START POSES //
    /////////////////

    public static Pose2d BLUE_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE - TILE, HALF_FIELD - HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
    public static Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, HALF_FIELD - HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
    public static Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, -HALF_FIELD + HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE - TILE, -HALF_FIELD + HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);

    ////////////////////
    // BACKDROP POSES //
    ////////////////////


  public static Pose2d RED_MIDDLE_OF_SPIKES = new Pose2d(-TILE-THREE_QUARTER_TILE, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
  public static Pose2d BLUE_MIDDLE_OF_SPIKES = new Pose2d(-TILE-THREE_QUARTER_TILE, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);

  //  public static Pose2d BLUE_BACKDROP_RIGHT = new Pose2d(2*TILE, 29.264, FACE_TOWARD_BACKSTAGE);
//  public static Pose2d RED_BACKDROP_LEFT = new Pose2d(2*TILE, -29.264, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_RIGHT = new Pose2d(2 * TILE, TILE + HALF_TILE - QUARTER_TILE+ 1, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_LEFT = new Pose2d(2 * TILE, -TILE - QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    //  public static Pose2d BLUE_BACKDROP_CENTER = new Pose2d(2*TILE, 35.264, FACE_TOWARD_BACKSTAGE);
//  public static Pose2d RED_BACKDROP_CENTER = new Pose2d(2*TILE, -35.264, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_CENTER = new Pose2d(2 * TILE, TILE + HALF_TILE+1.5, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_CENTER = new Pose2d(2 * TILE, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);

    //  public static Pose2d BLUE_BACKDROP_LEFT = new Pose2d(2*TILE, 41.264, FACE_TOWARD_BACKSTAGE);
//  public static Pose2d RED_BACKDROP_RIGHT = new Pose2d(2*TILE, -41.264, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_LEFT = new Pose2d(2 * TILE, TILE + HALF_TILE + QUARTER_TILE + 1, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_RIGHT = new Pose2d(2 * TILE, -TILE - TILE + QUARTER_TILE - .5, FACE_TOWARD_BACKSTAGE);

  public static Pose2d SUPER_RED_BACKDROP_RIGHT = new Pose2d(2 * TILE, -TILE - TILE + QUARTER_TILE + 2, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_RED_BACKDROP_LEFT = new Pose2d(2 * TILE, -TILE - QUARTER_TILE-2, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_BLUE_BACKDROP_RIGHT = new Pose2d(2 * TILE, TILE + HALF_TILE - QUARTER_TILE+2, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_BLUE_BACKDROP_LEFT = new Pose2d(2 * TILE, TILE + HALF_TILE + QUARTER_TILE-2, FACE_TOWARD_BACKSTAGE);

  public static Pose2d UP_THE_MID_BLUE_BACKDROP_RIGHT = new Pose2d(TILE, 29.264+3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d UP_THE_MID_RED_BACKDROP_LEFT = new Pose2d(TILE, -29.264-3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d UP_THE_MID_BLUE_BACKDROP_LEFT = new Pose2d(TILE, 41.264-3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d UP_THE_MID_RED_BACKDROP_RIGHT = new Pose2d(TILE, -41.264+3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d UP_THE_MID_BLUE_BACKDROP_CENTER = new Pose2d(TILE, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);
  public static Pose2d UP_THE_MID_RED_BACKDROP_CENTER = new Pose2d(TILE+QUARTER_TILE, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);



  //  public static Pose2d BLUE_BACKDROP_STAGING = new Pose2d(2*TILE-5, 35.264, FACE_TOWARD_BACKSTAGE);
//  public static Pose2d RED_BACKDROP_STAGING = new Pose2d(2*TILE-5, -35.264, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_STAGING = new Pose2d(2 * TILE - 10, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_STAGING = new Pose2d(2 * TILE - 10, TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);

    /////////////////
    // SPIKE POSES //
    /////////////////

    //Center Prop
    public static Pose2d RED_BACKSTAGE_SPIKE_C = new Pose2d(HALF_TILE, -TILE - HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_SPIKE_C_PAST = new Pose2d(HALF_TILE + 3, -TILE - QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_SPIKE_C_DROP = new Pose2d(HALF_TILE + 3, -TILE - QUARTER_TILE - 4, FACE_TOWARD_BLUE);

    public static Pose2d RED_AUDIENCE_SPIKE_C = new Pose2d(-TILE - HALF_TILE, -TILE - HALF_TILE, FACE_TOWARD_BLUE);

    public static Pose2d RED_AUDIENCE_SPIKE_C_PAST = new Pose2d(-TILE - HALF_TILE + 3, -TILE - QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_SPIKE_C_DROP = new Pose2d(-TILE - HALF_TILE + 3, -TILE - QUARTER_TILE - 4, FACE_TOWARD_BLUE);

    //Spike Right
    public static Pose2d RED_BACKSTAGE_SPIKE_R = new Pose2d(TILE - QUARTER_TILE-2, -TILE - HALF_TILE+2, FACE_45_DEGREES);
    public static Pose2d RED_BACKSTAGE_SPIKE_R_PAST = new Pose2d(TILE - QUARTER_TILE + 1, -TILE - HALF_TILE + 5, Math.toRadians(45));
    public static Pose2d RED_BACKSTAGE_SPIKE_R_DROP = new Pose2d(TILE - QUARTER_TILE - .5, -TILE - HALF_TILE + 3, Math.toRadians(45));

    public static Pose2d RED_AUDIENCE_SPIKE_R = new Pose2d(-TILE - QUARTER_TILE-1, -TILE - HALF_TILE, FACE_45_DEGREES);
    public static Pose2d RED_AUDIENCE_SPIKE_R_PAST = new Pose2d(-TILE - QUARTER_TILE + 2, -TILE - HALF_TILE + 2.5, Math.toRadians(65));
    public static Pose2d RED_AUDIENCE_SPIKE_R_DROP = new Pose2d(-TILE - QUARTER_TILE + 1.3, -TILE - HALF_TILE + 1, Math.toRadians(65));

    //Left Prop
    public static Pose2d RED_BACKSTAGE_SPIKE_L = new Pose2d(TILE - THREE_QUARTER_TILE + 2, -TILE - HALF_TILE, FACE_135_DEGREES);

    public static Pose2d RED_BACKSTAGE_SPIKE_L_PAST = new Pose2d(TILE - THREE_QUARTER_TILE, -TILE - HALF_TILE + 4.6, Math.toRadians(150));
    public static Pose2d RED_BACKSTAGE_SPIKE_L_DROP = new Pose2d(TILE - THREE_QUARTER_TILE + 2, -TILE - HALF_TILE + 4.6, Math.toRadians(150));

    public static Pose2d RED_AUDIENCE_SPIKE_L = new Pose2d(-TILE - THREE_QUARTER_TILE + 2, -TILE - HALF_TILE, FACE_135_DEGREES);


    public static Pose2d BLUE_BACKSTAGE_SPIKE_L = flipYAxis(RED_BACKSTAGE_SPIKE_R);

    public static Pose2d BLUE_AUDIENCE_SPIKE_L = flipYAxis(RED_AUDIENCE_SPIKE_R);

    public static Pose2d BLUE_AUDIENCE_SPIKE_R = flipYAxis(RED_AUDIENCE_SPIKE_L);

    public static Pose2d BLUE_BACKSTAGE_SPIKE_R = flipYAxis(RED_BACKSTAGE_SPIKE_L);

    public static Pose2d BLUE_BACKSTAGE_SPIKE_C = flipYAxis(RED_BACKSTAGE_SPIKE_C);

    public static Pose2d BLUE_AUDIENCE_SPIKE_C = flipYAxis(RED_AUDIENCE_SPIKE_C);

    ///////////////////////////////
    // NEUTRAL PIXEL STACK POSES //
    ///////////////////////////////

  public static Pose2d BLUE_OFFSET_NEUTRAL_PIXEL_STAGING = new Pose2d(-2 * TILE - QUARTER_TILE - 5, 34.969 + 4, FACE_TOWARD_BACKSTAGE);
  public static Pose2d RED_OFFSET_NEUTRAL_PIXEL_STAGING = new Pose2d(-2 * TILE - QUARTER_TILE - 5, -34.969 - 4, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_NEUTRAL_PIXEL_STAGEDOOR = new Pose2d(-64.281 + HALF_ROBOT_LENGTH, -11.656, FACE_TOWARD_BACKSTAGE);
  public static Pose2d RED_NEUTRAL_PIXEL_CENTERSPIKE = new Pose2d(-2 * TILE - QUARTER_TILE, -23.469+3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d RED_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP = new Pose2d(-2 * TILE - QUARTER_TILE - 3 - DISTANCE_TO_PIXEL_STACK_FROM_STAGING, -23.469+3, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_NEUTRAL_PIXEL_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_NEUTRAL_PIXEL_STAGEDOOR = new Pose2d(-64.281 + HALF_ROBOT_LENGTH, 11.656, FACE_TOWARD_BACKSTAGE);
  public static Pose2d BLUE_NEUTRAL_PIXEL_CENTERSPIKE = new Pose2d(-2 * TILE - QUARTER_TILE - 5, 23.469-3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d BLUE_NEUTRAL_PIXEL_CENTERSPIKE_PICKUP = new Pose2d(-2 * TILE - QUARTER_TILE - 5 -  DISTANCE_TO_PIXEL_STACK_FROM_STAGING, 23.469-3, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_NEUTRAL_PIXEL_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE-DISTANCE_TO_PIXEL_STACK_FROM_STAGING, TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_NEUTRAL_PIXEL_PICKUP_TOWARD_WALL_MORE = new Pose2d(-TILE * 2 - HALF_TILE-DISTANCE_TO_PIXEL_STACK_FROM_STAGING-4, TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_NEUTRAL_STAGING = new Pose2d(-2 * TILE + 10, TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);

  public static Pose2d RED_NEUTRAL_PIXEL_TRUSS = new Pose2d(-TILE * 2 - QUARTER_TILE, -34.969, FACE_TOWARD_BACKSTAGE);
  public static Pose2d RED_NEUTRAL_PIXEL_TRUSS_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE - DISTANCE_TO_PIXEL_STACK_FROM_STAGING, -34.969, FACE_TOWARD_BACKSTAGE);

  public static Pose2d BLUE_NEUTRAL_PIXEL_TRUSS = new Pose2d(-TILE * 2 - QUARTER_TILE, 34.969, FACE_TOWARD_BACKSTAGE);
  public static Pose2d BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE - DISTANCE_TO_PIXEL_STACK_FROM_STAGING, 34.969, FACE_TOWARD_BACKSTAGE);


  ///////////////////
    // Parking Poses //
    ///////////////////

    public static Pose2d BLUE_CORNER_PARK = new Pose2d(2 * TILE, TILE * 2 + HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_CORNER_PARK = new Pose2d(2 * TILE, -TILE * 2 - HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_MIDDLE_PARK = new Pose2d(2 * TILE, -HALF_TILE, FACE_135_DEGREES);
    public static Pose2d BLUE_MIDDLE_PARK = new Pose2d(2 * TILE, HALF_TILE, FACE_225_DEGREES);

    public static Pose2d RED_NEUTRAL_STAGING = new Pose2d(-2 * TILE + 10, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_BACKSTAGE_START_LANE_A = new Pose2d(TILE, HALF_FIELD - THREE_QUARTER_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKSTAGE_START_LANE_F = new Pose2d(TILE, -HALF_FIELD + THREE_QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_BACKSTAGE_ALIGNMENT = new Pose2d(TILE, TILE + HALF_TILE + 5, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_AUDIENCE_ALIGNMENT = new Pose2d(-TILE - HALF_TILE, TILE + HALF_TILE + 3, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_STAGEDOOR_ENTRANCE = new Pose2d(-QUARTER_TILE - TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_STAGEDOOR_ENTRANCE = new Pose2d(-QUARTER_TILE - TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_STAGEDOOR_EXIT = new Pose2d(HALF_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_STAGEDOOR_EXIT = new Pose2d(HALF_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_TRUSS = new Pose2d(-HALF_TILE, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_TRUSS = new Pose2d(-HALF_TILE, +TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_SPIKE_L_LINE = new Pose2d(-TILE * 2, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_SPIKE_R_LINE = new Pose2d(-TILE * 2, TILE + HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_SAFE_STRAFE = new Pose2d(-TILE * 2 - QUARTER_TILE, -TILE - HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_SAFE_STRAFE = new Pose2d(flipYAxis(new Vector2d(RED_SAFE_STRAFE.position.x, RED_SAFE_STRAFE.position.y)), FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_THROUGH_DOOR = new Pose2d(TILE + EIGHTH_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_THROUGH_DOOR = new Pose2d(TILE + EIGHTH_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    /////////////////
    // Super Poses //
    /////////////////
    public static Pose2d SUPER_BLUE_AUDIENCE_START_POSE = new Pose2d(-TILE - QUARTER_TILE, 2 * TILE + HALF_TILE + 1, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_BLUE_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, 2 * TILE + HALF_TILE + 1, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_RED_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, -2 * TILE - HALF_TILE -1, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_RED_AUDIENCE_START_POSE = new Pose2d(-TILE - QUARTER_TILE, -2 * TILE - HALF_TILE -1, FACE_TOWARD_BACKSTAGE);


    public static Pose2d SUPER_RED_NEUTRAL_PIXEL_TRUSS = new Pose2d(-TILE * 2 - QUARTER_TILE, -34.969+3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_RED_NEUTRAL_PIXEL_TRUSS_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE - DISTANCE_TO_PIXEL_STACK_FROM_STAGING, -34.969+.65, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_BLUE_NEUTRAL_PIXEL_TRUSS = new Pose2d(-TILE * 2 - QUARTER_TILE, 34.969-3, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_BLUE_NEUTRAL_PIXEL_TRUSS_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE - DISTANCE_TO_PIXEL_STACK_FROM_STAGING, 34.969-.65, FACE_TOWARD_BACKSTAGE);

    public static Pose2d SUPER_RED_STAGEDOOR_ENTRANCE = new Pose2d(-TILE - HALF_TILE - 5, -11.656, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_BLUE_STAGEDOOR_ENTRANCE = flipYAxis(SUPER_RED_STAGEDOOR_ENTRANCE);

    public static Pose2d SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR = new Pose2d(-2 * TILE - QUARTER_TILE - 8 + HALF_ROBOT_LENGTH, -11.656, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR = new Pose2d(-2 * TILE - QUARTER_TILE - 8 + HALF_ROBOT_LENGTH, 11.656, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_RED_NEUTRAL_PIXEL_STAGEDOOR_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE-DISTANCE_TO_STAGE_DOOR_PIXEL_STACK_FROM_STAGING, -11.656, FACE_TOWARD_BACKSTAGE);
  public static Pose2d SUPER_BLUE_NEUTRAL_PIXEL_STAGEDOOR_PICKUP = new Pose2d(-TILE * 2 - HALF_TILE-DISTANCE_TO_STAGE_DOOR_PIXEL_STACK_FROM_STAGING, 11.656, FACE_TOWARD_BACKSTAGE);

    public static Pose2d SUPER_RED_STAGEDOOR_BY_BACKDROP = new Pose2d(TILE + QUARTER_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d SUPER_BLUE_STAGEDOOR_BY_BACKDROP = new Pose2d(TILE + QUARTER_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);


    public static Pose2d flipYAxis(Pose2d pose) {
        Pose2d output = new Pose2d(pose.position.x, -pose.position.y, -pose.heading.log());
        return output;
    }

    public static Vector2d flipYAxis(Vector2d vector) {
        Vector2d output = new Vector2d(vector.x, -vector.y);
        return output;
    }

    public static Vector2d PoseToVector(Pose2d pose) {
        return new Vector2d(pose.position.x, pose.position.y);
    }
}