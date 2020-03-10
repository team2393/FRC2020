/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.RobotMap;

/** Robot code for testing hood solenoid */
public class SolenoidHoodTestRobot extends BasicRobot
{
  private final Solenoid hood_solenoid = new Solenoid(RobotMap.HOOD_ADJUST);
  private final Timer timer =  new Timer();

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.putBoolean("Hood Up", (hood_solenoid.get()));
  }

  @Override
  public void disabledInit()
  {
    super.disabledInit();
    hood_solenoid.set(false);
  }

  @Override
  public void teleopPeriodic()
  {
    // Hood solenoid is on as long as X button is held
    hood_solenoid.set(OI.isIntakeTogglePressed());  
    SmartDashboard.putBoolean("Hood Up", (hood_solenoid.get()));
  }

  @Override
  public void autonomousInit() 
  {
    super.autonomousInit();
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic()
  {
    //TODO not sure if I used the timer correctly 
    if(timer.get() >= 0.5)
    {
      timer.reset();
      timer.start();
      
      hood_solenoid.set(!hood_solenoid.get());
    }
    SmartDashboard.putBoolean("Hood Up", (hood_solenoid.get()));

  }
}