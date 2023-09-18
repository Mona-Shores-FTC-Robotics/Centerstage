// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package org.firstinspires.ftc.teamcode.ObjectClasses;

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

}