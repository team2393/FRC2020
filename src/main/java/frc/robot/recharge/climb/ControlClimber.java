/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Control Telescope and Climber motor */
public class ControlClimber extends CommandBase
{
  private final Climber climber;

  public ControlClimber(final Climber climber)
  {
    this.climber= climber;
    addRequirements(climber);
  }

  @Override
  public void execute()
  {
    climber.moveTelescope(OI.getTelescopeSpeed());
    climber.pullUp(OI.getClimbSpeed());
  }
}
