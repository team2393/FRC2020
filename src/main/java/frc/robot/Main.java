/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.recharge.CharacterizationRobot;
// import frc.robot.demo.motor.FalconTestRobot;
// import frc.robot.demo.motor.TrajectoryTestRobot;
// import frc.robot.demo.commands.CommandRobot;
import frc.robot.recharge.Enterprise;

/** 'main' class, selects which robot to run */
public final class Main
{
  public static void main(String... args)
  {
    RobotBase.startRobot(Enterprise::new);
  }
}
