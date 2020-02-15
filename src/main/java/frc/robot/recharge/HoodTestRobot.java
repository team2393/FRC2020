/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.shooter.Hood;

/** Robot code for testing hood */
public class HoodTestRobot extends BasicRobot
{
  private final Hood hood = new Hood();
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.putData("Hood PID", hood.getPID());
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Hood Angle", hood.getHoodAngle());   
  }

  @Override
  public void teleopPeriodic()
  {
    // Hold A button to 'home'
    if (OI.joystick.getAButton())
      hood.homeHood();

    // 'left/right' axis to directly run rotator angle motor
    hood.setAngleMotor(OI.getDirection());
  }

  @Override
  public void autonomousPeriodic()
  {
    // Every 3 seconds toggle between two angles
    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    hood.setHoodAngle(high ? 60 : 30);
  }
}
