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
import frc.robot.recharge.led.LEDStrip;

/** Robot code for testing devices */
public class TestRobot extends BasicRobot
{
  private final DigitalInput ball = new DigitalInput(8);
  private final LEDStrip led = new LEDStrip();

  @Override
  public void disabledPeriodic()
  {
    led.rainbow();
  }

  @Override
  public void teleopPeriodic()
  {
    SmartDashboard.putBoolean("ball detected", !ball.get());
    led.oscillate();
  }

  @Override
  public void autonomousPeriodic()
  {
    led.bluewhite();
  }
}
