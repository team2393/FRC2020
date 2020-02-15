/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodFar extends CommandBase {
  
  private final Hood hood;

  public HoodFar(Hood hood) 
  {
    this.hood = hood;
    addRequirements(hood);
  }

@Override
  public void initialize() 
  {
    hood.setHoodAngle(10);
  }

  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
