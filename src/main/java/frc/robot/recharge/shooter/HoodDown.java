/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HoodDown extends InstantCommand
{
  private final Hood hood;

  public HoodDown(Hood hood) 
  {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() 
  {  
    super.initialize();
    hood.set(false);
  }
}
