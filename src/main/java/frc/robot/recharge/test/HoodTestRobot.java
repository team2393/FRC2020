/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
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
    // 1) Move hood manually,check angle:
    //      0 degree = horizontal, out (more than ever used in practice)
    //     90 degree = vertical, up
    //   ~120 degree = Retracted all the way, "start" position
    SmartDashboard.putNumber("Hood Angle", hood.getHoodAngle());   
  }

  @Override
  public void teleopPeriodic()
  {
    // 2) 'forward' should move 'in',
    //    from fully out/horizontal towards the 'start' position.
    //    If not, motor.setInverted(true) and start over at step 1
    hood.setAngleMotor(OI.getSpeed());
  }

  @Override
  public void autonomousPeriodic()
  {
    // 3) Tune P, then D
    //    First with angles like 30, 60 which have the hood 'outside'.
    //    Then with angles like 110, 90 which are more realistic match angles.
    // Every 3 seconds toggle between two angles
    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    hood.setHoodAngle(high ? 60 : 30);
  }
}
