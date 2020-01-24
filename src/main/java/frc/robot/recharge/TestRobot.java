/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Robot code for testing devices */
public class TestRobot extends BasicRobot
{
  DigitalInput ball = new DigitalInput(8);

  @Override
  public void teleopPeriodic()
  {
    SmartDashboard.putBoolean("ball detected", !ball.get());
  }
}
