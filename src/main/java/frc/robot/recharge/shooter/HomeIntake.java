/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Home the 'intake'
 * 
 *  Raise intake until it hits the homing switch,
 *  which resets the angle encoder.
 */
public class HomeIntake extends CommandBase
{
  private final Intake intake;
  private boolean homed;

  public HomeIntake(final Intake intake)
  {
    this.intake = intake;
  }

  @Override
  public void execute()
  {
    homed = intake.homeIntake();
  }

  @Override
  public boolean isFinished()
  {
    return homed;
  }
}
