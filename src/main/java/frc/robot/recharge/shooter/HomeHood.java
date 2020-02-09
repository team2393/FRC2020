/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Home the 'hood'
 * 
 *  Move hood until it hits the homing switch,
 *  which resets the angle encoder.
 */
public class HomeHood extends CommandBase
{
  private final Hood hood;
  private boolean homed;

  public HomeHood(final Hood hood)
  {
    this.hood = hood;
  }

  @Override
  public void execute()
  {
    homed = hood.homeHood();
  }

  @Override
  public boolean isFinished()
  {
    return homed;
  }
}
