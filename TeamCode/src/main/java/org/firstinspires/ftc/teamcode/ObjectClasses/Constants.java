// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

  public static RobotType robot;

  public static void setRobot(RobotType type)
  {
      robot = type;
  }

  public enum RobotType {
        ROBOT_2023,
        ROBOT_CHASSIS,
        ROBOT_VISION,
        ROBOT_MECHANISM
  }

  public static RobotType getRobot() {
     if (robot == RobotType.ROBOT_2023) {
        return RobotType.ROBOT_2023;
      } else if (robot == RobotType.ROBOT_VISION)
     {
         return RobotType.ROBOT_VISION;
     } else if (robot == RobotType.ROBOT_MECHANISM){
         return RobotType.ROBOT_MECHANISM;
     }
     else {
        return RobotType.ROBOT_CHASSIS;
      }
  }

    double ROBOT_LENGTH = 18.0;
    double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2;

    double HALF_FIELD = 72.0;
    double TILE = 23.5;
    double HALF_TILE = TILE/2;

    double FACE_TOWARD_RED = Math.toRadians(0);
    double FACE_TOWARD_BLUE = Math.toRadians(180);
    double FACE_TOWARD_FRONTSTAGE = Math.toRadians(270);
    double FACE_TOWARD_BACKSTAGE = Math.toRadians(90);

    Pose2d BLUE_BACKDROP = new Pose2d(-TILE-HALF_TILE, 2*TILE, FACE_TOWARD_BACKSTAGE);
    Pose2d RED_BACKDROP = new Pose2d(mirrorAcrossXAxis(BLUE_BACKDROP), FACE_TOWARD_BACKSTAGE);

    Pose2d BLUE_LEFT_START_POSE = new Pose2d(-HALF_FIELD + HALF_ROBOT_LENGTH,HALF_TILE, FACE_TOWARD_RED);
    Pose2d BLUE_RIGHT_START_POSE = new Pose2d(-HALF_FIELD + HALF_ROBOT_LENGTH, -HALF_TILE-TILE, FACE_TOWARD_RED);
    Pose2d RED_RIGHT_START_POSE = new Pose2d(HALF_FIELD - HALF_ROBOT_LENGTH, HALF_TILE, FACE_TOWARD_BLUE);
    Pose2d RED_LEFT_START_POSE = new Pose2d(+HALF_FIELD - HALF_ROBOT_LENGTH,-HALF_TILE - TILE, FACE_TOWARD_BLUE);

    Pose2d RED_RIGHT_SPIKE_LOCATION = new Pose2d(TILE+HALF_TILE,HALF_TILE, FACE_TOWARD_BLUE);
    Pose2d BLUE_LEFT_SPIKE_LOCATION = new Pose2d(mirrorAcrossXAxis(RED_RIGHT_SPIKE_LOCATION), FACE_TOWARD_RED);

    Pose2d RED_LEFT_SPIKE_LOCATION = new Pose2d(RED_RIGHT_SPIKE_LOCATION.position.x, RED_RIGHT_SPIKE_LOCATION.position.y-(TILE*2), FACE_TOWARD_BLUE);
    Pose2d BLUE_RIGHT_SPIKE_LOCATION =  new Pose2d(mirrorAcrossXAxis(RED_LEFT_SPIKE_LOCATION), FACE_TOWARD_RED);

    public static Vector2d mirrorAcrossXAxis(Pose2d input) {
        Vector2d output = new Vector2d(-input.position.x, input.position.y);
        return output;
    }

}