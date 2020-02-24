/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodAngle extends CommandBase
{  
  private final Hood hood;

  private double angle;

  public HoodAngle(Hood hood, double angle) 
  {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() 
  {
    hood.setHoodAngle(angle);
  }

  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
