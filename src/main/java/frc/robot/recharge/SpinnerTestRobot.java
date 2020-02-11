/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.shooter.Spinner;

/** Robot code for testing spinner */
public class SpinnerTestRobot extends BasicRobot
{
  private final Spinner spinner = new Spinner();
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("kV", 0.0239);
    SmartDashboard.setDefaultNumber("P", 0.01);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("RPM", spinner.getRPM());
  }

  @Override
  public void teleopPeriodic()
  {
    // +- 12 Volts
    final double voltage = (OI.getSpeed() * 12);
    System.out.println("Voltage: " + (voltage) + " RPM: " + spinner.getRPM());
    spinner.setVoltage(voltage);
  }

  @Override
  public void autonomousPeriodic()
  {
    // TODO Tune PID, then pick some reasonable RPM values between which to toggle
    spinner.configure(SmartDashboard.getNumber("kV", 0),
                      SmartDashboard.getNumber("P", 0));
    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    spinner.setRPM(high ? 2000 : 3000);
  }
}
