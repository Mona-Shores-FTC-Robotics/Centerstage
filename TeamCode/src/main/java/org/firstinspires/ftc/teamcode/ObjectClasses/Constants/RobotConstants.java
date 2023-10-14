// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package org.firstinspires.ftc.teamcode.ObjectClasses.Constants;

public class RobotConstants {



  public static RobotType robot;

  public static void setRobot(RobotType type)
  {
      robot = type;
  }

  public enum RobotType {
        ROBOT_CENTERSTAGE,
        ROBOT_ARM_END_EFFECTOR,
        ROBOT_CHASSIS,
        ROBOT_VISION,
        ROBOT_VISION_FAST_MOTORS,
        ROBOT_MOTOR_TEST_MECHANISM
  }

  public static RobotType getRobot() {
     if (robot == RobotType.ROBOT_CENTERSTAGE) {
        return RobotType.ROBOT_CENTERSTAGE;
      } else if (robot == RobotType.ROBOT_VISION)
     {
         return RobotType.ROBOT_VISION;
     } else if (robot == RobotType.ROBOT_VISION_FAST_MOTORS)
     {
         return RobotType.ROBOT_VISION_FAST_MOTORS;
     }
     else if (robot == RobotType.ROBOT_MOTOR_TEST_MECHANISM){
         return RobotType.ROBOT_MOTOR_TEST_MECHANISM;
     }
     else {
        return RobotType.ROBOT_CHASSIS;
      }
  }




}