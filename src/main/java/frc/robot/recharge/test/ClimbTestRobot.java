/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.climb.Climber;

/** Robot code for testing climb */
public class ClimbTestRobot extends BasicRobot
{
  private final Climber climber = new Climber();
  
  @Override
  public void teleopPeriodic()
  {
    climber.moveTelescope(OI.getSpeed());

    climber.pullUp(OI.getWheelSpeed());
  }
}
